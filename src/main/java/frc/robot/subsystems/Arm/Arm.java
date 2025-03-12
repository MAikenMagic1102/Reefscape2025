// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants;

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

  boolean closedLoop = false;

  private double targetPosition = 0;
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

    //armMotor.setPosition(armCaNcoder.getAbsolutePosition().getValueAsDouble() / ArmConstants.armRotorToSensor);
    motorSim = armMotor.getSimState();
    cancoderSim = armCaNcoder.getSimState();
    armCaNcoder.setPosition(0);
  }

  @Override
  public void periodic() {

    if(armAtScoring()){
      ArmConstants.driveSpeed = 0.3;
    }else{
      ArmConstants.driveSpeed = 1.0;
    }
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm Angle", getAngleDegrees());
    Logger.recordOutput("Arm At Goal", atGoal());

    Logger.recordOutput("Arm Motor Voltage", armMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Arm Stator Current", armMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Arm Motor Temp", armMotor.getDeviceTemp().getValueAsDouble());
    
    if(closedLoop){
      Logger.recordOutput("Arm Setpoint", armMotor.getClosedLoopReference().getValueAsDouble());
    }
  }

  public boolean armHalfScored(){
    return getAngleDegrees() < -100;
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
    return Units.rotationsToDegrees(armCaNcoder.getPosition().getValueAsDouble() / ArmConstants.armGearingCANcoder);
  }

  public boolean armAtScoring(){
    return getAngleDegrees() < -180;
  }
  
  public boolean atGoal(){
    return Math.abs(targetPosition - getAngleDegrees()) < ArmConstants.positionTolerence;
  }

  public void setOpenLoop(double demand){
    dutyOut.withOutput(demand);
    armMotor.setControl(dutyOut);
    closedLoop = false;
  }

  public void setAnglePosition(double angle){
    targetPosition = angle;
    posVoltage.withPosition(Units.degreesToRotations(angle));
    armMotor.setControl(posVoltage);
    closedLoop = true;
  }

  public Command setAngle(double angle){
    return runOnce(() -> setAnglePosition(angle));
  }
  

}