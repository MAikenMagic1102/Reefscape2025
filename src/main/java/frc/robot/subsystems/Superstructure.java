// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Mechanism.SuperStructureMechanism;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator = new Elevator ();

  private final SuperStructureMechanism mech = new SuperStructureMechanism();
  
  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.periodic();
    mech.update(elevator.getPositionMeters());
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
}
