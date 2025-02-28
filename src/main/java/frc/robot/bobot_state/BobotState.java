package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.varc.BargeTagTracker;
import frc.robot.bobot_state.varc.HPSTagTracker;
import frc.robot.bobot_state.varc.ReefTagTracker;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.util.VirtualSubsystem;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
  
/**
 * Class full of static variables and methods that store robot state we'd need across mulitple
 * subsystems. It's called {@link #BobotState} as to not conflict with WPILib's {@link
 * edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d();

  private static ReefTagTracker reefTracker = new ReefTagTracker();
  private static HPSTagTracker hpsTracker = new HPSTagTracker();
  private static BargeTagTracker bargeTracker = new BargeTagTracker();

  private final NetworkTable bobotStateTable = NetworkTableInstance.getDefault().getTable("BobotState");
  
  // Reef tag data
  private final DoublePublisher reefAnglePublisher = bobotStateTable.getDoubleTopic("ReefTargetDegrees").publish();
  private final DoublePublisher reefAnglePublisherRad = bobotStateTable.getDoubleTopic("ReefTargetRadians").publish();
  private final IntegerPublisher reefTargetFiducialId = bobotStateTable.getIntegerTopic("ReefTagId").publish();
  
  // Barge tag data
  private final DoublePublisher bargeAnglePublisher = bobotStateTable.getDoubleTopic("BargeTargetDegrees").publish();
  private final DoublePublisher bargeAnglePublisherRad = bobotStateTable.getDoubleTopic("BargeTargetRadians").publish();
  private final IntegerPublisher bargeTargetFiducialId = bobotStateTable.getIntegerTopic("BargeTagId").publish();

  // HPS data
  private final BooleanPublisher closeToHumanPlayer =  bobotStateTable.getBooleanTopic("CloseToHumanPlayer").publish();


  public static void offerVisionObservation(PoseObservation observation) {
    BobotState.poseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getVisionObservations() {
    return BobotState.poseObservations;
  }


  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  public static Rotation2d getRotationToClosestReef() {
    return BobotState.reefTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestHPS() {
    return BobotState.hpsTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestBarge() {
    return BobotState.bargeTracker.getRotationTarget();
  }

  public static double getDistanceMetersFromClosestHPS() {
    return BobotState.hpsTracker.getDistanceMeters();
  }

  public static Trigger nearHumanPlayer() {
    return new Trigger(() -> BobotState.hpsTracker.getDistanceMeters() < 2);
  }

  public static Trigger humanPlayerShouldReady() {
    return new Trigger(() -> BobotState.hpsTracker.getDistanceMeters() < 0.5);
  }

  @Override
  public void periodic() {

    {
      // Update Trackers
      reefTracker.update();
      bargeTracker.update();

      // Update Network Tables

      // Reef Tag tracking
      // Tag angle in Degrees
      reefAnglePublisher.set(reefTracker.getRotationTarget().getDegrees());
      // Tag angle in Radians
      reefAnglePublisherRad.set(reefTracker.getRotationTarget().getRadians());
      // Tag ID
      reefTargetFiducialId.set(FieldUtils.getClosestReef().tag.fiducialId());

      // Barge tag tracking
      // Tag angle in Degrees
      bargeAnglePublisher.set(bargeTracker.getRotationTarget().getDegrees());
      // Tag angle in Radians
      bargeAnglePublisherRad.set(bargeTracker.getRotationTarget().getRadians());
      // Tag ID
      bargeTargetFiducialId.set(FieldUtils.getBargeTag().fiducialId());

      // Displays true when distance to HPS is less than 2 meters
      closeToHumanPlayer.set(this.nearHumanPlayer().getAsBoolean());
    }

    {
      hpsTracker.update();

      String calcLogRoot = logRoot + "HPS/";
      SmartDashboard.putNumber(logRoot + "Something?", 0.0);
    }

    {
      bargeTracker.update();

      String calcLogRoot = logRoot + "Barge/";
    }
  }

  @Override
  public void simulationPeriodic() {}
}
