package frc.robot.subsystems.can_range;
public class CanRangeIOSim implements CanRangeIO {
  public void updateInputs(CanRangeIOInputs inputs) {
    inputs.connected = true;
    inputs.distanceMeters = 1.0;
  }
}
