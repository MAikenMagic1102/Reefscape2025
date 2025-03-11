// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
    public class ElevatorConstants {
    public static String bus = "can2";
    public static int motorLID = 21;
    public static int motorRID = 22;

    public static double elevatorGearing = 5.4;
    public static double elevatorMass = Units.lbsToKilograms(10);
    public static double elevatorPullyRadius = Units.inchesToMeters(1.125);
    public static double elevatorMinHeightMeters = Units.inchesToMeters(1);
    public static double elevatorMaxHeightMeters = Units.inchesToMeters(60);
    public static double elevatorStartingHeightMeters = Units.inchesToMeters(1);
    public static double elevatorPullyCircum = Math.PI*2*ElevatorConstants.elevatorPullyRadius;

    public static double conversion = (Math.PI * elevatorPullyRadius * 2) / elevatorGearing;
    public static Distance elevatorPullyRadiusDistance = Inches.of(1.128);

    public static double elevatorTolerance = 0.07;
    
    public static double setHome = 0;
    public static double hpLoad = 0;
    public static double reefL1 = 0;    
    public static double reefL2 = 0;
    public static double reefL3 = 0;
    public static double reefL4 = 0.83;
    public static double ALGAE = 0.0;
    public static double idle = 0;
    public static double safePos = 0.35;

    public static double driveSpeed = 1.0;

    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
            
        )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            
        )
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold((1.15 / elevatorPullyCircum))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0.0)
                .withReverseSoftLimitEnable(true)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(13.0)
                .withKD(0.5)
                .withKG(0.4)
                .withKV(7.2)
        )        
        .withSlot1(
            new Slot1Configs()
                .withKG(0.310)
                .withKV(0.305)
                .withKA(0.05)
                .withKP(92)
               
        )
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(60)
                .withMotionMagicCruiseVelocity(50)
                .withMotionMagicJerk(1000)
        )
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(5.4)
        );
    public static Object elevatorPullyRadiusDistances;

    
}
