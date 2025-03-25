// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX motorL;
  private TalonFX motorR;

  private TalonFXSimState motorSim;
  private DCMotor elevatorGearbox = DCMotor.getKrakenX60Foc(2);

  private ElevatorSim ElevatorSim = 
  new ElevatorSim(
          elevatorGearbox,
          ElevatorConstants.elevatorGearing,
          ElevatorConstants.elevatorMass,
          ElevatorConstants.elevatorPullyRadius,
          ElevatorConstants.elevatorMinHeightMeters,
          ElevatorConstants.elevatorMaxHeightMeters,
          true,
          ElevatorConstants.elevatorStartingHeightMeters);

  private DutyCycleOut dutyCycleOutput = new DutyCycleOut(0);
  private PositionVoltage posVoltage = new PositionVoltage(0).withSlot(0);
  private MotionMagicVoltage mmPosition = new MotionMagicVoltage(0).withSlot(1);

  double kG;
  double kV;
  double kA;
  double kP;

  private double targetPosition = 0;

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

    SmartDashboard.putNumber("elevatorKG", 0.0);

    SmartDashboard.putNumber("elevatorKV", 0.0);

    SmartDashboard.putNumber("elevatorKA", 0.0);
   
    SmartDashboard.putNumber("elevatorKP", 0.0);

    motorSim = motorL.getSimState();
    motorL.setPosition(0);
  }

  @Override
  public void periodic() {
    if(aboveHalf()){
      ElevatorConstants.driveSpeed = 0.3;
    }else{
      ElevatorConstants.driveSpeed = 1.0;
    }
    // This method will be called once per scheduler run
    //SmartDashboard
    Logger.recordOutput("Elevator/ at Goal", atGoal());
    Logger.recordOutput("Elevator/ above Intake", aboveIntake());
    Logger.recordOutput("Elevator/ above Half", aboveHalf());
    Logger.recordOutput("Elevator/ Position Meters", getPositionMeters());
    
    Logger.recordOutput("Elevator/ Motors/ LeftMotor/ Velocity", motorL.getVelocity().getValueAsDouble());
    Logger.recordOutput("Elevator/ Motors/ LeftMotor/ Acceleration", motorL.getAcceleration().getValueAsDouble());

    Logger.recordOutput("Elevator/ Motors/ LeftMotor/ Voltage", motorL.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Elevator/ Motors/ LeftMotor/ Stator Current", motorL.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator/ Motors/ LeftMotor/ Temp", motorL.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Elevator/ Motors/ RightMotor/ Voltage", motorR.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Elevator/ Motors/ RightMotor/ Stator Current", motorR.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator/ Motors/ RightMotor/ Temp", motorR.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    ElevatorSim.setInput(motorSim.getMotorVoltage());

    ElevatorSim.update(0.02);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(ElevatorSim.getCurrentDrawAmps()));

    motorSim.setRawRotorPosition(ElevatorSim.getPositionMeters() / ElevatorConstants.conversion);
    motorSim.setRotorVelocity(ElevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.conversion);
  }

  public boolean atGoal() {
    return Math.abs(targetPosition - getPositionMeters()) < ElevatorConstants.elevatorTolerance;
  }

  public boolean aboveIntake(){
    return getPositionMeters() > 0.24;
  }

  public boolean aboveHalf(){
    return getPositionMeters() > 0.5;
  }

  public boolean belowHalf(){

    return getPositionMeters() < 0.5;
  }

  public double getPositionMeters() {
    return rotationsToMeters(motorL.getRotorPosition().getValue()).in(Meters);
  }

  public void setPositionMeters(double height) {
    targetPosition = height;
    posVoltage.withPosition(height / ElevatorConstants.elevatorPullyCircum);
    motorL.setControl(posVoltage);
  }

  public void setPositionMetersMM(double height) {
    targetPosition = height;
    mmPosition.withPosition(height / ElevatorConstants.elevatorPullyCircum);
    motorL.setControl(mmPosition);
  }

  public Command setHeight(double height){
    return runOnce(() -> setPositionMetersMM(height));
  }

  public void setOpenLoop(double input){
    dutyCycleOutput.withOutput(input);

    motorL.setControl(dutyCycleOutput);
  }

  public void setHome(){

  }

  public double getGoalPos(){
    return 0;
  }

  public static Distance rotationsToMeters(Angle rotations) {
    /* Apply gear ratio to input rotations */
    var gearedRadians = rotations.in(Radians) / ElevatorConstants.elevatorGearing;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return ElevatorConstants.elevatorPullyRadiusDistance.times(gearedRadians);
  }

  public static Angle metersToRotations(Distance meters) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(Meters) / ElevatorConstants.elevatorPullyRadiusDistance.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return Radians.of(wheelRadians * ElevatorConstants.elevatorGearing);
  }

  public static  LinearVelocity rotationsToMetersVel(AngularVelocity rotations) {
    /* Apply gear ratio to input rotations */
    var gearedRotations = rotations.in(RadiansPerSecond) / ElevatorConstants.elevatorGearing;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return ElevatorConstants.elevatorPullyRadiusDistance.per(Second).times(gearedRotations);
  }

  public static  AngularVelocity metersToRotationsVel(LinearVelocity meters) {
    /* Divide the distance by the wheel radius to get radians */
    var wheelRadians = meters.in(MetersPerSecond) / ElevatorConstants.elevatorPullyRadiusDistance.in(Meters);
    /* Then multiply by gear ratio to get rotor rotations */
    return RadiansPerSecond.of(wheelRadians * ElevatorConstants.elevatorGearing);
  }

}
