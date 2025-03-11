package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.field.FieldUtils;

public class ReefTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget = Rotation2d.kZero;

  public void update() {
    rotationTarget =
        FieldUtils.getClosestReef().tag.pose().getRotation().toRotation2d().plus(Rotation2d.kPi);
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }
}
