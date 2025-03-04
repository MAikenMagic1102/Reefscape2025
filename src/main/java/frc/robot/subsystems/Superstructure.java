// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Mechanism.SuperStructureMechanism;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator = new Elevator ();
    private Arm arm = new Arm();

  private final SuperStructureMechanism mech = new SuperStructureMechanism();
  
  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.periodic();
    arm.periodic();
    mech.update(elevator.getPositionMeters());
  }

  public boolean elevatorAtGoal(){
    return elevator.atGoal();
  }

  public Command runElevatorUp() {
    return new InstantCommand(() -> elevator.setOpenLoop(0.25));
  }
  public Command runElevatorDown(){
    return new InstantCommand(() -> elevator.setOpenLoop(-0.25));
  }

  public Command stopElevator() {
    return new InstantCommand(() -> elevator.setOpenLoop(0));
  }

  public Command setMiddlePos() {
     return new InstantCommand(() -> elevator.setPositionMetersMM(0.85));
 }

 public Command setHighPos(){
  return new InstantCommand(() -> elevator.setPositionMeters(1));
 }  
 public Command setL1Pos() {
  return new InstantCommand(() -> elevator.setPositionMeters(0.5));
 }
 public Command setL2Pos() {
  return new InstantCommand(() -> elevator.setPositionMeters(1)); 
 }
 public Command setL3Pos() {
  return new InstantCommand(() -> elevator.setPositionMeters(1.5));
 }
 public Command setL4Pos() {
  return new InstantCommand(() -> elevator.setPositionMeters(2));
 }
 public Command setTestHome () {
  return new InstantCommand(()-> elevator.setPositionMetersMM(0.005));
 }

 public Command armUp() {
  return runOnce(() -> arm.setOpenLoop(0.4));
}

public Command armDown() {
  return runOnce(() -> arm.setOpenLoop(-0.4));
}

public Command armStop(){
  return runOnce(() -> arm.setOpenLoop(0.0));
}

public Command setArmAngle(double angle){
  return runOnce(() -> arm.setAnglePosition(angle));
}

 }
