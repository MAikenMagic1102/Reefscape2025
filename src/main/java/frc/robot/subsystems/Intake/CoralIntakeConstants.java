// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class CoralIntakeConstants {
    public static String bus = "rio";
    public static int pivotMotorID = 21;
    public static int rollerMotorID = 22;
    

    public static double cIntakeGearing = 0.0;
    public static double cIntakeMass = Units.lbsToKilograms(10);
    public static double cIntakeMinHeightMeters = Units.inchesToMeters(1);
    public static double cIntakeMaxHeightMeters = Units.inchesToMeters(10);
    public static double cIntakeStartingHeightMeters = Units.inchesToMeters(1);

    public static double setHome = 0;
    public static double hpLoad = 0;
    public static double floorIntake = 0;
    public static double stationIntake = 0;
    public static double stowedPos = 0;
    public static double idle = 0;
    public static double safePos = 0;

    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withStatorCurrentLimit(35)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withFeedback(
            new FeedbackConfigs()
            .withSensorToMechanismRatio(cIntakeGearing)
        );


}
