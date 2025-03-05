// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotor;
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
