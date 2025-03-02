// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX armMotor;
  private TalonFXSimState motorSim;
  private CANcoderSimState cancoderSim;

  private CANcoder armCaNcoder;

  private DCMotor armGearbox = DCMotor.getKrakenX60(1);

  private SingleJointedArmSim armSim = 
    new SingleJointedArmSim(
      armGearbox,
      ArmConstants.armGearing,
      SingleJointedArmSim.estimateMOI(ArmConstants.armLength, ArmConstants.armMass),
      ArmConstants.armLength,
      ArmConstants.armMinAngle,
      ArmConstants.armMaxAngle,
      true,
      ArmConstants.armStartingAngle);

  private DutyCycleOut dutyOut = new DutyCycleOut(0);
  private PositionVoltage posVoltage = new PositionVoltage(0).withSlot(0);
  private MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0).withSlot(0);

/** Creates a new Arm. */
  public Arm() {
    armMotor = new TalonFX(ArmConstants.motorID, ArmConstants.busname);
    armCaNcoder = new CANcoder(ArmConstants.cancoderID, ArmConstants.busname);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = armMotor.getConfigurator().apply(ArmConstants.config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Retry config apply up to 5 times, report if failure */
    StatusCode ccstatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      ccstatus = armCaNcoder.getConfigurator().apply(ArmConstants.ccconfig);
      if (ccstatus.isOK()) break;
    }
    if(!ccstatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + ccstatus.toString());
    }

    armMotor.setPosition(Units.degreesToRotations(0));
    motorSim = armMotor.getSimState();
    cancoderSim = armCaNcoder.getSimState();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle", getAngleDegrees());
    SmartDashboard.putNumber("Motor Position", armMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Motor Rotor Position", armMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("CC Position", armCaNcoder.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    cancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    armSim.setInput(motorSim.getMotorVoltage());

    armSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    motorSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads()) * ArmConstants.armGearing);
    cancoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads()) * ArmConstants.armGearingCANcoder);
  }

  public double getAngleDegrees(){
    return Units.rotationsToDegrees(armMotor.getRotorPosition().getValueAsDouble()) / ArmConstants.armGearing;
  }
  
  public boolean atGoal(){
    return false;
  }

  public void setOpenLoop(double demand){
    dutyOut.withOutput(demand);
    armMotor.setControl(dutyOut);
  }
}
