// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

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


/** Add your docs here. */
public class ArmConstants {
    public static String busname = "rio";
    public static int motorID = 31;
    public static int cancoderID = 32;

public static double gearRatio = 0.0;
public static double armLength = 0.0;
public static double armMass = 0.0;
public static double armInAngle = 0.0;
public static double armMaxAngle = 0.0;
public static double startingAngle =0.0;

public static double armToL1 = 0.0;
public static double armToL2 = 0.0;
public static double armToL3 =0.0;
public static double armToL4 =0.0;
public static double armToHome =0.0;
public static double armToHalf =0.0;
public static double armToFull =0.0;


    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(70)
                .withStatorCurrentLimit(120)

        )  
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)

          )   
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(cancoderID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(0.0)
                .withSensorToMechanismRatio(0.0)
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

    public static CANcoderConfiguration cc_cfg = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(0.0)
        );
  

}
