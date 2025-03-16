// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.Climber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotor;
  private double targetPosition;

  private PositionVoltage posVoltage = new PositionVoltage(0).withSlot(0);
  
  public Climber() {
    climberMotor = new TalonFX(ClimberConstants.motorID, ClimberConstants.rioBus);
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for ( int i = 0; i < 5; ++i){
      status = climberMotor.getConfigurator().apply(ClimberConstants.config);
      if (status.isOK()) break;
    }

    if (!status.isOK()){
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void atAngle(double angleDeg){
    climberMotor.setControl(new PositionVoltage(angleDeg));
  }

  public void setClimberPosition(double angleDeg) {
    // targetPosition = angleDeg;
    new PositionVoltage(Units.degreesToRotations(angleDeg));
    // PositionVoltage.withPosition(Units.degreesToRadians(angleDeg));
    // posVoltage.withPosition(ClimberConstants.targetPosition.Units.degreesToRadians());
    SmartDashboard.putNumber("ClimberVoltageOut", climberMotor.getMotorVoltage().getValueAsDouble());
  }

  public boolean atGoal(){
    return false;
  }

  public double getClimberAngle(){
    return Units.rotationsToDegrees(climberMotor.getPosition().getValueAsDouble());
  }

  public double getError(double targetPosition){
    return Math.abs(getClimberAngle() - targetPosition);
  }

  public void setOpenLoop(double climberOutput){
    climberMotor.setControl(new DutyCycleOut(climberOutput));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("isClimberAtGoal", getClimberAngle());
    // This method will be called once per scheduler run
  }
}
