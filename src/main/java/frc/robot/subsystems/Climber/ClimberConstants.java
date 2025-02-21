// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

/** Add your docs here. */
public class ClimberConstants {
    public static int motorID = 46;
    string bus = "rio";
    TalonFXConfiguration config = new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs()
                            .withNeutralModel(NeutralMode.Brake)
                        );
}
