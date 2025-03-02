// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4finalPos extends SequentialCommandGroup {
  /** Creates a new L4finalPos. */
  public L4finalPos(Superstructure struct) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      struct.setL1Pos(),
      new WaitUntilCommand(struct::elevatorAtGoal),
      new WaitCommand(1),
      struct.setL2Pos(),
      new WaitUntilCommand(struct::elevatorAtGoal),
      new WaitCommand(1),
      struct.setL3Pos(),
      new WaitUntilCommand(struct::elevatorAtGoal),
      new WaitCommand(1),
      struct.setL4Pos(),
      new WaitUntilCommand(struct::elevatorAtGoal),
      new WaitCommand(1),
      struct.setTestHome()
    );
  }
}
