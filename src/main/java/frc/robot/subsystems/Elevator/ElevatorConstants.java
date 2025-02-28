// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
    public class ElevatorConstants {
    public static String bus = "rio";

    public static int motorLID = 21;
    public static int motorRID = 22;

    public static double kElevatorGearing = 5.4;
    public static double kCarriageMass = Units.lbsToKilograms(15);
    public static double kElevatorDrumRadius = Units.inchesToMeters(1.128);
    public static double kMinElevatorHeightMeters = Units.inchesToMeters(0);
    public static double kMaxElevatorHeightMeters = Units.inchesToMeters(54.42);
    public static double kStartingHeight = Units.inchesToMeters(0);

    public static Distance kElevatorDrumRadiusDistance = Inches.of(1.128);
    
    public static double kElevatorDrumCircumfrence = (Math.PI * kElevatorDrumRadius * 2);
    public static double conversion = (Math.PI * kElevatorDrumRadius * 2) / kElevatorGearing;

    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(70)
                .withStatorCurrentLimit(120)      
        )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
        )
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(5.4)
        )
        .withSlot0(
            new Slot0Configs()
                .withKG(0.310)
                .withKV(0.305)
                .withKA(0.05)
                .withKP(1)
        )
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.3)
                .withMotionMagicAcceleration(0.3)
        );
    
}