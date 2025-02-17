// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
    public class ElevatorConstants {
    public static String bus = "rio";

    public static int motorLID = 21;
    public static int motorRID = 22;

    public static double kElevatorGearing = 6.5;
    public static double kCarriageMass = Units.lbsToKilograms(20);
    public static double kElevatorDrumRadius = Units.inchesToMeters(1);
    public static double kMinElevatorHeightMeters = Units.inchesToMeters(0);
    public static double kMaxElevatorHeightMeters = Units.inchesToMeters(54.42);
    public static double kStartingHeight = Units.inchesToMeters(0);

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
                .withSensorToMechanismRatio(6.5)
        );
    
}