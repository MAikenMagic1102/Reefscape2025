// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
    public class ElevatorConstants {
    public static String bus = "rio";
    public static int motorLID = 21;
    public static int motorRID = 22;

    public static double elevatorGearing = 1/5.4;
    public static double elevatorMass = Units.lbsToKilograms(10);
    public static double elevatorPullyRadius = Units.inchesToMeters(1);
    public static double elevatorMinHeightMeters = Units.inchesToMeters(1);
    public static double elevatorMaxHeightMeters = Units.inchesToMeters(40);
    public static double elevatorStartingHeightMeters = Units.inchesToMeters(1);
    public static double elevatorPullyCircum = Math.PI*2*ElevatorConstants.elevatorPullyRadius;

    public static double setHome = 0;
    public static double hpLoad = 0;
    public static double reefL1 = 0;    
    public static double reefL2 = 0;
    public static double reefL3 = 0;
    public static double reefL4 = 0;
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
                .withSensorToMechanismRatio(5.4)
        );

    
}
