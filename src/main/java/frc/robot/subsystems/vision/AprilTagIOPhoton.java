package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimationMethod;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagIOPhoton implements AprilTagIO {
  protected final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final Transform3d robotToCamera;

  public AprilTagIOPhoton(VisionSource source) {

    camera = new PhotonCamera(source.name());

    // Holds the position of the cam relative to the center of the robot
    robotToCamera = source.robotToCamera();

    // Estimates the pose
    estimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            source.robotToCamera());

    // Sets the strategy to be used when there is only one tag detected
    estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  //
  public void updateInputs(AprilTagIOInputs inputs) {
    List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();

    // List<PoseObservation> allLocalizedPoseObservations = new ArrayList<>();
    // List<PhotonTrackedTarget> allTargets = List.of();
    List<Translation2d> validCorners = new ArrayList<>();
    List<Translation2d> rejectedCorners = new ArrayList<>();

    List<Integer> validIds = new ArrayList<>();
    List<Integer> rejectedIds = new ArrayList<>();

    List<PoseObservation> validPoseObservations = new ArrayList<>();
    List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

    List<Pose3d> validPoses = new ArrayList<>();
    List<Pose3d> rejectedPoses = new ArrayList<>();

    List<Pose3d> validAprilTagPoses = new ArrayList<>();
    List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

    // Loops through the results from the latest PhotonPiplelineResult
    for (PhotonPipelineResult result : unreadResults) {
      // Filtering of tags by ambiguity threshold & validity of id
      // Does not apply to global pose estimation
      // List<PhotonTrackedTarget> filteredTargets =
      // result.getTargets().stream()
      // .filter(
      // target ->
      // target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff
      // && target.getFiducialId() != -1)
      // .toList();

      // allTargets.addAll(targets);

      // Detected Corners
      for (PhotonTrackedTarget target : result.getTargets()) {
        // Rejects if target is not valid, i.e.
        // - Target is too far away
        // - Ambiguity is greater than 0.05
        if (AprilTagAlgorithms.isValid(target)) {
          // for (TargetCorner corner : target.getDetectedCorners()) {
          //   validCorners.add(new Translation2d(corner.x, corner.y));
          // }

          // Maps all detected corners onto a translation2d
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              // Adds the translation 2ds to an array of valid corners
              .forEach(validCorners::add);

          // Adds the id of the target to an array of the valid ids
          validIds.add(target.getFiducialId());

          // Adds the poses of the target to an array of the valid target poses
          validAprilTagPoses.add(
              VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get());
        } else { // The target is invalid
          // for (TargetCorner corner : target.getDetectedCorners()) {
          //   rejectedCorners.add(new Translation2d(corner.x, corner.y));
          // }

          // Maps the target's corners to a translation2d and adds it to a list of rejected corners
          target.getDetectedCorners().stream()
              .map(corner -> new Translation2d(corner.x, corner.y))
              .forEach(rejectedCorners::add);

          // Adds the id of the target to the list of rejected targets
          rejectedIds.add(target.getFiducialId());

          // Adds the pose of the target to a list of rejected poses
          if (target.getFiducialId() != -1) {
            VisionConstants.fieldLayout
                .getTagPose(target.getFiducialId())
                .ifPresent(rejectedAprilTagPoses::add);
          }
        }
      }

      // List<PoseObservation> localizedPoseObservations =
      // filteredTargets.stream()
      // .map(
      // target -> {
      // Transform3d robotToTarget =
      // target.getBestCameraToTarget().plus(robotToCamera);
      // Pose3d tagPose =
      // VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).get();
      // Pose3d localizedRobotPose = tagPose.transformBy(robotToTarget.inverse());
      // // TODO: account for gyro
      //
      // return new PoseObservation(
      // localizedRobotPose,
      // result.getTimestampSeconds(),
      // target.getPoseAmbiguity(),
      // // new int[] {target.getFiducialId()}
      // target.getFiducialId(),
      // PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      // })
      // .toList();

      // allLocalizedPoseObservations.addAll(localizedPoseObservations);

      // Pose Estimation
      Optional<EstimatedRobotPose> maybeEstimatedPose = estimator.update(result);

      if (!maybeEstimatedPose.isPresent()) {
        continue;
      }

      // Estimates the pose of the robot based on the pipeline result
      // Note: Remember, we are still looping through our list of pipeline results
      EstimatedRobotPose estimatedPose = maybeEstimatedPose.get();

      // If a multi-tag result can be found
      if (result.getMultiTagResult().isPresent()) {
        // Gets and stores the multi-tag result
        MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();

        // Legit estimates pose
        Pose3d pose = estimatedPose.estimatedPose;

        // Gets standard deviation as defined in the AprilTagAlgorithms
        Matrix<N3, N1> stdDevs =
            AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());

        // Records our pose estimation
        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                multiTagResult.estimatedPose.ambiguity,
                // multiTagResult.fiducialIDsUsed.stream().mapToInt(id -> id).toArray()
                -100,
                stdDevs,
                PoseEstimationMethod.MULTI_TAG);

        // Adds our result to the srray of valid pose observations
        validPoseObservations.add(observation);
        validPoses.add(observation.robotPose());
      } else if (!result.getTargets().isEmpty()) { // No multitag result (only one target seen?)
        PhotonTrackedTarget target = result.getTargets().get(0);

        Pose3d pose = estimatedPose.estimatedPose;

        // Gets deviation
        Matrix<N3, N1> stdDevs =
            AprilTagAlgorithms.getEstimationStdDevs(pose.toPose2d(), result.getTargets());
        PoseObservation observation =
            new PoseObservation(
                estimatedPose.estimatedPose,
                estimatedPose.timestampSeconds,
                target.poseAmbiguity,
                // new int[] {target.fiducialId}
                target.fiducialId,
                stdDevs,
                PoseEstimationMethod.SINGLE_TAG);

        // If we have a valid target
        if (AprilTagAlgorithms.isValid(target)) {
          // Add pose to list of valid poses
          validPoseObservations.add(observation);
          validPoses.add(observation.robotPose());
        } else { // Invalid target
          // Add pose to list of invalid poses
          rejectedPoseObservations.add(observation);
          rejectedPoses.add(observation.robotPose());
        }
      }
    }

    // I dont even know anymore, this stuff is driving me actually insane.
    // IS IT SO HARD TO ADD COMMENTS WHILE YOU ARE WRITING CODE?!?!?!?
    // Its really handy, and you can make little todos for yourself, like
    // TODO: Learn how to write comments, moron
    /* You
    can
    even
    do
    multi
    line
    comments,
    like
    this!

    All you need are forward slashes and astericks!
    VS code has a shortcut for commenting: Ctrl + ?
    */

    inputs.connected = camera.isConnected();

    // Converts our matricies to arrays
    inputs.validCorners = validCorners.toArray(Translation2d[]::new);
    inputs.rejectedCorners = rejectedCorners.toArray(Translation2d[]::new);

    inputs.validIds = validIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.rejectedIds = rejectedIds.stream().mapToInt(Integer::intValue).toArray();

    inputs.validPoseObservations = validPoseObservations.toArray(PoseObservation[]::new);
    inputs.rejectedPoseObservations = rejectedPoseObservations.toArray(PoseObservation[]::new);

    inputs.validPoses = validPoses.toArray(Pose3d[]::new);
    inputs.rejectedPoses = rejectedPoses.toArray(Pose3d[]::new);

    inputs.validAprilTagPoses = validAprilTagPoses.toArray(Pose3d[]::new);
    inputs.rejectedAprilTagPoses = rejectedAprilTagPoses.toArray(Pose3d[]::new);
  }
}
