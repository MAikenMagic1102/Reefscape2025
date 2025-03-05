// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

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
     public static int motorID = 46;
     public static String rioBus = "rio";

     
//    TalonFX( 46 ,"rio")
//    motor.apply(config);
//     TalonFXConfiguration config = new TalonFXConfiguration()
//     //                    .withMotorOutput(
//    //                         new MotorOutputConfigs()
//    //                         .withNeutralModel(NeutralMode.Brake)
//    //                     );
//    DutyCycleOutput(0)
//     void setOpenLoop(90){
//         DutyCycle.withOutput(90)
//         motor.applyControl(Dutycycle);
//     }


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
            .withFeedbackRemoteSensorID(0)
            .withRotorToSensorRatio(0)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    )
    .withSlot0(
        new Slot0Configs()
            .withKG(0)
            .withKD(0)
            .withKP(0)
            .withKV(0)
    );

public static CANcoderConfiguration ccconfig = new CANcoderConfiguration()
    .withMagnetSensor(
        new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(0.0)
            .withMagnetOffset(0)
            //.withMagnetOffset(1.32)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );
}

