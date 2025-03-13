package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.Util.PoseUtils;

public class ReefFace {
  public final AprilTagStruct tag;
  public final ReefPole leftPole;
  public final ReefPole rightPole;

  public ReefFace(AprilTagStruct tag) {
    this.tag = tag;
    this.leftPole = new ReefPole(tag, -FieldConstants.tagToReef);
    this.rightPole = new ReefPole(tag, FieldConstants.tagToReef);
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
