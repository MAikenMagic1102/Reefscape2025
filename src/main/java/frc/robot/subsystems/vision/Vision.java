package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.vision.AprilTagIO.AprilTagIOInputs;
import frc.robot.subsystems.vision.VisionConstants.AprilTagCameraConfig;
import frc.robot.util.MagicVirtualSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class Vision extends MagicVirtualSubsystem {
  public static record AprilTagCamera(
      AprilTagIO io,
      AprilTagIOInputsAutoLogged inputs,
      VisionSource source,
      Alert disconnectedAlert) {}

  private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  private static final String aggregateAprilTagLogRoot = "Subsystems/Vision";
  private static final String aprilTagLogRoot = "RealOutputs/Subsystems/Vision/Cameras";
  private static final String validAggregateLoggingPath = aggregateAprilTagLogRoot + "/Valid";
  private static final String rejectedAggregateLoggingPath = aggregateAprilTagLogRoot + "/Rejected";

  public Vision() {
    for (AprilTagCameraConfig config : VisionConstants.aprilTagCamerasConfigs) {
      AprilTagIO io;

      switch (Constants.currentMode) {
        case REAL:
          io = new AprilTagIOPhoton(config.source());
          break;
        case SIM:
          io = new AprilTagIOPhotonSim(config.source(), config.simConfig());
          break;
        case REPLAY:
        default:
          io = new AprilTagIO() {};
          break;
      }

      Alert disconnectedAlert =
          new Alert(
              aprilTagLogRoot + " " + config.source().name() + " is disconnected!",
              AlertType.kWarning);

      aprilTagCameras.add(
          new AprilTagCamera(
              io, new AprilTagIOInputsAutoLogged(), config.source(), disconnectedAlert));
    }
  }

  @Override
  public void periodic() {
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

    for (AprilTagCamera cam : aprilTagCameras) {
      cam.io.updateInputs(cam.inputs);
      Logger.processInputs(aprilTagLogRoot + "/" + cam.source.name(), cam.inputs);

      cam.disconnectedAlert.set(!cam.inputs.connected);

      validCorners.addAll(Arrays.asList(cam.inputs.validCorners));
      rejectedCorners.addAll(Arrays.asList(cam.inputs.rejectedCorners));

      validIds.addAll(Arrays.stream(cam.inputs.validIds).boxed().toList());
      rejectedIds.addAll(Arrays.stream(cam.inputs.rejectedIds).boxed().toList());

      validPoseObservations.addAll(Arrays.asList(cam.inputs.validPoseObservations));
      rejectedPoseObservations.addAll(Arrays.asList(cam.inputs.rejectedPoseObservations));

      validPoses.addAll(Arrays.asList(cam.inputs.validPoses));
      rejectedPoses.addAll(Arrays.asList(cam.inputs.rejectedPoses));

      validAprilTagPoses.addAll(Arrays.asList(cam.inputs.validAprilTagPoses));
      rejectedAprilTagPoses.addAll(Arrays.asList(cam.inputs.rejectedAprilTagPoses));

      for (PoseObservation observation : cam.inputs.validPoseObservations) {
        BobotState.offerVisionObservation(observation);
      }

      // if (cam.source.name() == "frontLeftCam") {
      //   Logger.recordOutput(validAggregateLoggingPath + "/LeftCamValidPoseObservations", cam.inputs.validPoses);
      //   // flCamPose.set(cam.inputs.validPoses);
      //   // flTargets.set(cam.inputs.validAprilTagPoses);
      // } else if (cam.source.name() == "frontRightCam") {
      //   Logger.recordOutput(validLoggingPath + "/RightCamValidPoseObservations", cam.inputs.validPoses);
      //   // frCamPose.set(cam.inputs.validPoses);
      //   // frTargets.set(cam.inputs.validAprilTagPoses);
      // }
    }

    Logger.recordOutput(
        validAggregateLoggingPath + "/ValidCorners", validCorners.toArray(Translation2d[]::new));
    Logger.recordOutput(
        rejectedAggregateLoggingPath + "/RejectedCorners",
        rejectedCorners.toArray(Translation2d[]::new));

    Logger.recordOutput(
        validAggregateLoggingPath + "/ValidIds",
        validIds.stream().mapToInt(Integer::intValue).toArray());
    Logger.recordOutput(
        rejectedAggregateLoggingPath + "/RejectedIds",
        rejectedIds.stream().mapToInt(Integer::intValue).toArray());

    Logger.recordOutput(
        validAggregateLoggingPath + "/ValidPoseObservations",
        validPoseObservations.toArray(PoseObservation[]::new));
    Logger.recordOutput(
        rejectedAggregateLoggingPath + "/RejectedPoseObservations",
        rejectedPoseObservations.toArray(PoseObservation[]::new));

    Logger.recordOutput(
        validAggregateLoggingPath + "/ValidPoses", validPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        rejectedAggregateLoggingPath + "/RejectedPoses", rejectedPoses.toArray(Pose3d[]::new));

    Logger.recordOutput(
        validAggregateLoggingPath + "/ValidAprilTagPoses",
        validAprilTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        rejectedAggregateLoggingPath + "/RejectedAprilTagPoses",
        rejectedAprilTagPoses.toArray(Pose3d[]::new));
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> aprilTagSim.update(BobotState.getGlobalPose()));
  }
}

// public class Vision extends MagicVirtualSubsystem {
//   public static record AprilTagCamera(
//       AprilTagIO io,
//       AprilTagIOInputs inputs,
//       VisionSource source,
//       Alert disconnectedAlert) {}

//   private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

//   private static final String aprilTagLogRoot = "AprilTagCamera";
//   private static final String aggregateAprilTagLogRoot = "AggregateAprilTagCameras";
//   private final NetworkTable visionStateTable = NetworkTableInstance.getDefault().getTable("VisionState");

//   private final StructArrayPublisher<Pose3d> flCamPose = visionStateTable.getStructArrayTopic("FLCam", Pose3d.struct).publish();
//   private final StructArrayPublisher<Pose3d> frCamPose = visionStateTable.getStructArrayTopic("FRCam", Pose3d.struct).publish();
  
//   private final StructArrayPublisher<Pose3d> flTargets = visionStateTable.getStructArrayTopic("FLTargets", Pose3d.struct).publish();
//   private final StructArrayPublisher<Pose3d> frTargets = visionStateTable.getStructArrayTopic("FRTargets", Pose3d.struct).publish();


//   public Vision() {
//     for (AprilTagCameraConfig config : VisionConstants.aprilTagCamerasConfigs) {
//       AprilTagIO io;

//       switch (Constants.currentMode) {
//         case REAL:
//           io = new AprilTagIOPhoton(config.source());
//           break;
//         case SIM:
//           io = new AprilTagIOPhotonSim(config.source(), config.simConfig());
//           break;
//         case REPLAY:
//         default:
//           io = new AprilTagIO() {};
//           break;
//       }

//       Alert disconnectedAlert =
//           new Alert(
//               aprilTagLogRoot + " " + config.source().name() + " is disconnected!",
//               AlertType.kWarning);

//       aprilTagCameras.add(
//           new AprilTagCamera(
//               io, new AprilTagIOInputs(), config.source(), disconnectedAlert));
//     }
//   }

//   @Override
//   public void periodic() {
//     List<Translation2d> validCorners = new ArrayList<>();
//     List<Translation2d> rejectedCorners = new ArrayList<>();

//     List<Integer> validIds = new ArrayList<>();
//     List<Integer> rejectedIds = new ArrayList<>();

//     List<PoseObservation> validPoseObservations = new ArrayList<>();
//     List<PoseObservation> rejectedPoseObservations = new ArrayList<>();

//     List<Pose3d> validPoses = new ArrayList<>();
//     List<Pose3d> rejectedPoses = new ArrayList<>();

//     List<Pose3d> validAprilTagPoses = new ArrayList<>();
//     List<Pose3d> rejectedAprilTagPoses = new ArrayList<>();

//     for (AprilTagCamera cam : aprilTagCameras) {
//       cam.io.updateInputs(cam.inputs);
//       // Logger.processInputs(aprilTagLogRoot + "/" + cam.source.name(), cam.inputs);

//       cam.disconnectedAlert.set(!cam.inputs.connected);

//       validCorners.addAll(Arrays.asList(cam.inputs.validCorners));
//       rejectedCorners.addAll(Arrays.asList(cam.inputs.rejectedCorners));

//       validIds.addAll(Arrays.stream(cam.inputs.validIds).boxed().toList());
//       rejectedIds.addAll(Arrays.stream(cam.inputs.rejectedIds).boxed().toList());

//       validPoseObservations.addAll(Arrays.asList(cam.inputs.validPoseObservations));
//       rejectedPoseObservations.addAll(Arrays.asList(cam.inputs.rejectedPoseObservations));

//       validPoses.addAll(Arrays.asList(cam.inputs.validPoses));
//       rejectedPoses.addAll(Arrays.asList(cam.inputs.rejectedPoses));

//       validAprilTagPoses.addAll(Arrays.asList(cam.inputs.validAprilTagPoses));
//       rejectedAprilTagPoses.addAll(Arrays.asList(cam.inputs.rejectedAprilTagPoses));
//       SmartDashboard.putBoolean(aprilTagLogRoot + "/" + cam.source.name(), cam.inputs.connected);

//       for (PoseObservation observation : cam.inputs.validPoseObservations) {
//         BobotState.offerVisionObservation(observation);
//       }
      
//       // TODO: figure out why this doesn't work and fix it!
//       if (cam.source.name() == "frontLeftCam") {
//         flCamPose.set(cam.inputs.validPoses);
//         flTargets.set(cam.inputs.validAprilTagPoses);
//       } else if (cam.source.name() == "frontRightCam") {
//         frCamPose.set(cam.inputs.validPoses);
//         frTargets.set(cam.inputs.validAprilTagPoses);
//       }

//     }
    
//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/ValidCorners", validCorners.toArray(Translation2d[]::new));
//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/RejectedCorners",
//         rejectedCorners.toArray(Translation2d[]::new));

//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/ValidIds",
//         validIds.stream().mapToInt(Integer::intValue).toArray());
//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/RejectedIds",
//         rejectedIds.stream().mapToInt(Integer::intValue).toArray());

//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/ValidPoseObservations",
//         validPoseObservations.toArray(PoseObservation[]::new));
//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/RejectedPoseObservations",
//         rejectedPoseObservations.toArray(PoseObservation[]::new));

//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/ValidPoses", validPoses.toArray(Pose3d[]::new));
//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/RejectedPoses", rejectedPoses.toArray(Pose3d[]::new));

//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/ValidAprilTagPoses",
//         validAprilTagPoses.toArray(Pose3d[]::new));
//     Logger.recordOutput(
//         aggregateAprilTagLogRoot + "/RejectedAprilTagPoses",
//         rejectedAprilTagPoses.toArray(Pose3d[]::new));
//   }

//   @Override
//   public void simulationPeriodic() {
//     VisionConstants.aprilTagSim.ifPresent(
//         aprilTagSim -> aprilTagSim.update(BobotState.getGlobalPose()));
//   }
// }
