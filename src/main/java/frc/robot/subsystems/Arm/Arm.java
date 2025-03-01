// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX armMotor;
  private CANcoder arm_cc;

/** Creates a new Arm. */
  public Arm() {
    armMotor = new TalonFX(ArmConstants.motorID, ArmConstants.busname);
    arm_cc = new CANcoder(ArmConstants.cancoderID, ArmConstants.busname);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      armMotor.getConfigurator().apply(ArmConstants.config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Retry config apply up to 5 times, report if failure */
    StatusCode cc_status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      arm_cc.getConfigurator().apply(ArmConstants.cc_cfg);
      if (cc_status.isOK()) break;
    }
    if(!cc_status.isOK()) {
      System.out.println("Could not apply configs, error code: " + cc_status.toString());
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm position", getPosition());
  }

  public boolean atGoal(){
    return false;
  }

public double getPosition(){
  return arm_cc.getPosition().getValueAsDouble();
}

public void setAngle(){
  
}

public void setOpenLoop(double output){
  armMotor.setControl(new DutyCycleOut(output));
}

void setHome(){
  armMotor.setPosition(0);
}

double getGoalPos(){
  return 0.0;
}

public Command runOpenLoop(double output) {
  return runOnce(() -> setOpenLoop(output));
}

}