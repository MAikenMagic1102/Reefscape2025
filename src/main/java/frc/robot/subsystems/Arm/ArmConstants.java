// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmConstants {
    public static String busname = "rio";
    public static int motorID = 31;
    public static int cancoderID = 32;

    public static double armGearing = 89.83;
    public static double armGearingCANcoder = 3.5;
    public static double armLength = Units.inchesToMeters(25);
    public static double armMass = Units.lbsToKilograms(5.0);
    public static double armMinAngle = Units.degreesToRadians(-255.0);
    public static double armMaxAngle = Units.degreesToRadians(255.0);
    public static double armStartingAngle = Units.degreesToRadians(0.0);

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
          //10:70 - 18:66 S - 10:35
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(3.5)
                .withFeedbackRemoteSensorID(cancoderID)
                .withRotorToSensorRatio(25.67)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        );

    public static CANcoderConfiguration ccconfig = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.0)
                .withMagnetOffset(Units.degreesToRotations(180))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );

}
