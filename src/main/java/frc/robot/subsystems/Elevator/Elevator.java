// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX motorL;
  private TalonFX motorR;

  private TalonFXSimState motorL_sim;

  private DCMotor elevatorGearbox = DCMotor.getKrakenX60(2);

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
        elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMinElevatorHeightMeters,
          false,
          ElevatorConstants.kStartingHeight);   
  
  private DutyCycleOut dutyOut = new DutyCycleOut(0);

  /** Creates a new Elevator. */
  public Elevator() {
    motorL = new TalonFX(ElevatorConstants.motorLID, ElevatorConstants.bus);
    motorR = new TalonFX(ElevatorConstants.motorRID, ElevatorConstants.bus);

    motorR.setControl(new Follower(ElevatorConstants.motorLID, true)); 

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motorL.getConfigurator().apply(ElevatorConstants.config);
      status = motorR.getConfigurator().apply(ElevatorConstants.config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    motorL_sim = motorL.getSimState();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {

    motorL_sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Sim Battery Volts", RobotController.getBatteryVoltage());
    // This method will be called once per scheduler run during simulation
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(motorL_sim.getMotorVoltage());
    SmartDashboard.putNumber("motorL_sim Volts", motorL_sim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    SmartDashboard.putNumber("Elevator Pos Meters", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Elevator Pos Meters Per Second", m_elevatorSim.getVelocityMetersPerSecond());
    motorL_sim.setRawRotorPosition(m_elevatorSim.getPositionMeters() / ElevatorConstants.conversion);
    motorL_sim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.conversion);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    SmartDashboard.putNumberArray("Elevator Simulator Output", m_elevatorSim.getOutput().getData());
  }

  public double getSimPositionMeters(){
    return m_elevatorSim.getPositionMeters();
  }

  public double getPositionMeters(){
    //Convert rotor position to meters..
    return rotationsToMeters(motorL.getPosition().getValue()).in(Meters);
  }

  public void setOpenLoop(double demand){
    dutyOut.withOutput(demand);
    motorL.setControl(dutyOut);
  }

  private Distance rotationsToMeters(Angle rotations) {
    /* Apply gear ratio to input rotations */
    var gearedRadians = rotations.in(Radians) / ElevatorConstants.kElevatorGearing;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return ElevatorConstants.kElevatorDrumRadiusDistance.times(gearedRadians);
  }

  private Angle metersToRotations(Distance meters) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(Meters) / ElevatorConstants.kElevatorDrumRadiusDistance.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return Radians.of(wheelRadians * ElevatorConstants.kElevatorGearing);
  }

  private LinearVelocity rotationsToMetersVel(AngularVelocity rotations) {
    /* Apply gear ratio to input rotations */
    var gearedRotations = rotations.in(RadiansPerSecond) / ElevatorConstants.kElevatorGearing;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return ElevatorConstants.kElevatorDrumRadiusDistance.per(Second).times(gearedRotations);
  }

  private AngularVelocity metersToRotationsVel(LinearVelocity meters) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(MetersPerSecond) / ElevatorConstants.kElevatorDrumRadiusDistance.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return RadiansPerSecond.of(wheelRadians * ElevatorConstants.kElevatorGearing);
  }
}
