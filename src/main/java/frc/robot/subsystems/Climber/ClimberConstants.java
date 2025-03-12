// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ClimberConstants {
     public static final double climberTolerance = 0;
    public static int motorID = 51;
     public static String rioBus = "can2";

    
public static double armGearing = 0;
public static double armGearingCANcoder = 0;
public static double armRotorToSensor = 0;
public static double armLength = Units.inchesToMeters(0);
public static double armMass = Units.lbsToKilograms(0);
public static double armMinAngle = Units.degreesToRadians(0);
public static double armMaxAngle = Units.degreesToRadians(0);
public static double armStartingAngle = Units.degreesToRadians(0.0);

public static double positionTolerence = 0;

public static double reefL1 = 0;
public static double reefL2 = 0;
public static double reefL3 = 0;
public static double reefL4 = 0;

public static TalonFXConfiguration config = new TalonFXConfiguration()
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(70)
            .withStatorCurrentLimit(120)

    )  
    .withMotorOutput(
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)

      )   
      //10:70 - 18:66 S - 10:35
    .withFeedback(
        new FeedbackConfigs()
            .withSensorToMechanismRatio(0)
    )
    .withSlot0(
        new Slot0Configs()
            .withKG(0)
            .withKD(0)
            .withKP(0)
            .withKV(0)
    );
}
