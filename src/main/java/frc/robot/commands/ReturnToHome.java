// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CoralGripper.CoralGripper;
import frc.robot.subsystems.Intake.CoralIntake;
import frc.robot.subsystems.Intake.CoralIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnToHome extends SequentialCommandGroup {
  /** Creates a new ReturnToHome. */
  public ReturnToHome(Superstructure  superstructure, CoralIntake intake, CoralGripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new InstantCommand(() -> gripper.setEject()),

      intake.setAngleCommand(CoralIntakeConstants.floorIntake).alongWith(superstructure.setElevatorToScoreSafe()),

      new WaitUntilCommand(superstructure::isElevatorSafe),

      superstructure.setArmToHome(),

      new WaitUntilCommand(superstructure::isArmAtGoal),

      superstructure.setElevatorHome(),

      new InstantCommand(() -> gripper.setStop())

    );
  }
}
