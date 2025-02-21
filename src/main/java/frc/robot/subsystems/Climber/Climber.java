0
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotor;
  public Climber() {
    climberMotor = new TalonFX(ClimberConstants.motorID, ClimberConstants.bus)
    TalonFXConfiguration climberConfigs = new ClimberConstants.TalonFXConfiguration
  }

  public void setAngle(double angleDeg){
  climberMotor.setAngle(angleDeg);
  }

  public double getAngle(){
    return climberMotor.getAngle();
  }

  public double getError(double targetPosition){
    return Math.abs(getAngle() - targetPosition);
  }

  public void setOpenLoop(double climberOutput){
    climberMotor.setControl(climberOutput);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("climberPosition", climberMotor.getAngle);
    // This method will be called once per scheduler run
  }
}
