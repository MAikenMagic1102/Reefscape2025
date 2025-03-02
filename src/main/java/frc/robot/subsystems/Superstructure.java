// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Mechanism.SuperStructureMechanism;

public class Superstructure extends SubsystemBase {
  private Elevator elevator = new Elevator();
  private Arm arm = new Arm();

  private final SuperStructureMechanism structMechanism = new SuperStructureMechanism();
  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.periodic();
    arm.periodic();

    structMechanism.updateElevator(elevator.getPositionMeters());
    structMechanism.updateArm(arm.getAngleDegrees());
  }

  @Override
  public void simulationPeriodic(){
    elevator.simulationPeriodic();
    arm.simulationPeriodic();
  }

  public Command runElevatorUp() {
    return new InstantCommand(() -> elevator.setOpenLoop(0.15));
  }
  public Command runElevatorDown(){
    return new InstantCommand(() -> elevator.setOpenLoop(-0.15));
  }

  public Command stopElevator() {
    return new InstantCommand(() -> elevator.setOpenLoop(0));
  }

  public Command middlePosition(){
    return runOnce(() -> elevator.setPositionMeters(0.5));
  }

  public Command armUp() {
    return runOnce(() -> arm.setOpenLoop(0.1));
  }

  public Command armDown() {
    return runOnce(() -> arm.setOpenLoop(-0.1));
  }

  public Command armStop(){
    return runOnce(() -> arm.setOpenLoop(0.0));
  }
}
