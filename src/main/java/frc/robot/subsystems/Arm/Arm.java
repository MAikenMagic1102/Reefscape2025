// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX armMotor;
/** Creates a new Arm. */
  public Arm() {
    armMotor = new TalonFX(ArmConstants.motorID, ArmConstants.busname);

        /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = armMotor.getConfigurator().apply(ArmConstants.config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public boolean atGoal(){
    return false;
  }
}
