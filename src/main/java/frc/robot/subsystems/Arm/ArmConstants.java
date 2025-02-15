// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

/** Add your docs here. */
public class ArmConstants {
    public static String busname = "rio";
    public static int motorID = 1;
    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withStatorCurrentLimit(15)

        )  
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)

          )   
        .withFeedback(
            new FeedbackConfigs()
                .withSensorMechanismTatio(5.0)

        );

}
