// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.counter.UpDownCounter;

/** Add your docs here. */
public class CoralIntakeConstants {
    public static String bus = "rio";
    public static int pivotMotorID = 13;
    public static int rollerMotorID = 14;
    
    public static double armLength = 0.0;
    public static double gearRatio = 0.0;
    public static double armMass = 0.0;
    public static double armMinAngle = 0.0;
    public static double armMaxAngle = 0.0;
    public static double sartingAngle = 0.0;
    public static double intakeOut = 0.0;
    public static double intakeIn = 0.0;
    public static double up = 0.0;
    public static double down = 0.0;


    public static double cIntakeGearing = 5.0;
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
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimit(120)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            // .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withFeedback(
            new FeedbackConfigs()
            .withSensorToMechanismRatio(cIntakeGearing)
        )
        
        .withSlot0(
            new Slot0Configs()
            .withKG(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)



        );


}
