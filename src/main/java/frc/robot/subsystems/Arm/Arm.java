// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;6

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFx armMotor;
/** Creates a new Arm. */
  public Arm() {
    armMotor = new TalonFX(ArmConstants.motorID, ArmConstants.busname)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean atGoal(){
    return false;
  }
public double getAngle(){
  return 0.0;
}
public void setAngle(){
  
}
public void setOpenLoop(){

}
void setHome(){

}
double getGoalPos(){
  return 0.0;
}
void periodic(){
  SmartDashboard
  logging statements;
}
}
