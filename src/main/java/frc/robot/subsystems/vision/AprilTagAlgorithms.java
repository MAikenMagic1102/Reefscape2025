package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.Arrays;


public class AprilTagAlgorithms {

  // isValid returns true if the abiguity of the PhotonTrackedTarget argument is less than the
  // VisionConstant
  // for the ambiguity cutoff
  public static boolean isValid(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff
        && VisionConstants.goodIds.indexOf(target.getFiducialId()) != -1;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets The targets used in the calc for the pose.
   * @return The calculated standard deviations. Or empty if not suitable for estimation.
   * @apiNote Calc is short for calculator by the way.
   * @apiNote I'm just using slang guys.
   */
  public static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    // Used for average distance calculation
    int numTags = 0;
    // Will hold the average distance from the robots to the tags
    double avgDistance = 0;

    // Loops through all of the photon tracked targets
    for (PhotonTrackedTarget target : targets) {

      // tagPose holds the tag position of the current target
      var tagPose = VisionConstants.fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;

      // Increments numTags
      numTags++;
      // Adds the distance of the current tag to avgDistance for average distance calculation later
      avgDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    Matrix<N3, N1> stdDevs = VisionConstants.singleTagStdDevs;

    // If no tags detected, then
    if (numTags == 0) {
      // Return the standard deviation from VisionConstants
      return stdDevs;
    }

    // Calculate average distance
    avgDistance /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
      stdDevs = VisionConstants.multiTagStdDevs;
    }

    // Increase std devs based on average distance
    if (numTags == 1 && avgDistance > VisionConstants.singleTagPoseCutoffMeters) {
      // Too far for only one tag, throw away
      stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else { // We have multiple tags or our one tag is within the accepted distance
      // Multiplies all elements of stdDevs by (1 + (distance**2 / 30))?
      stdDevs = stdDevs.times(1 + (avgDistance * avgDistance / 30.0));
    }

    return stdDevs;
  }
}
