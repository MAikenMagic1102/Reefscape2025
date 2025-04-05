// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.lang.ModuleLayer.Controller;

import javax.security.auth.login.LoginException;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.CoralIntakeConstants;
import frc.robot.subsystems.can_range.CanRange;
import frc.robot.subsystems.can_range.CanRangeIO;
import frc.robot.subsystems.can_range.CanRangeIOReal;

public class CoralIntake extends SubsystemBase {

  private TalonFX rollerMotor;
  private TalonFX pivotMotor;

  private TalonFXSimState rollerSimState;
  private TalonFXSimState pivotSimState;

  private DCMotor intakeGearbox = DCMotor.getKrakenX60Foc(1);

  private CanRange coralSensor;

  private CanRangeIO canRangeIO = new CanRangeIOReal(43, false);
 
  private SingleJointedArmSim CoralIntakeSim = 
  new SingleJointedArmSim(
        intakeGearbox,
        CoralIntakeConstants.gearRatio,
        SingleJointedArmSim.estimateMOI(CoralIntakeConstants.armLength, CoralIntakeConstants.armMass),
        CoralIntakeConstants.armLength,
        CoralIntakeConstants.armMinAngle,
        CoralIntakeConstants.armMaxAngle,
        false,
        CoralIntakeConstants.armStartingAngle);

  private DutyCycleOut rollerOut = new DutyCycleOut(0);

  private DutyCycleOut pivotOut = new DutyCycleOut(0);
  private PositionVoltage posVoltage = new PositionVoltage(0).withSlot(0);
  private boolean isClosedLoop = false;

  private boolean L1Mode = false;

  private double targetPosition = 0;

  private double intakeSpeed = 0;


  // AdvantageScope log paths
  private final String loggerPath = "Subsystems/Intake";
  private final String coralSensorLoggerPath = loggerPath + "/CoralSensor";
  private final String motorLoggerPath = loggerPath + "/Motors";
  private final String rollerMotorLoggerPath = motorLoggerPath + "/Roller";
  private final String pivotMotorLoggerPath = motorLoggerPath + "/Pivot";
  private Debouncer intakeDebouncer;

  private boolean hasCoral = false;

  
  
  /** Creates a new CoralIntake. */
  public CoralIntake() {
    rollerMotor = new TalonFX(CoralIntakeConstants.rollerMotorID, CoralIntakeConstants.bus);
    pivotMotor = new TalonFX(CoralIntakeConstants.pivotMotorID, CoralIntakeConstants.bus);
    coralSensor = new CanRange("CoralSensor", canRangeIO);
    intakeDebouncer = new Debouncer(0.1);

    //Roller
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for ( int i = 0; i < 5; ++i){
      status = rollerMotor.getConfigurator().apply(CoralIntakeConstants.config);
      if (status.isOK()) break;
    }

    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    //Pivot
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for ( int i = 0; i < 5; ++i){
      status2 = pivotMotor.getConfigurator().apply(CoralIntakeConstants.config);
      if (status2.isOK()) break;
    }

    if (!status2.isOK()) {
      System.out.println("Could not apply configs, error code: " + status2.toString());
    }

    pivotMotor.setPosition(Units.degreesToRotations(92));

    rollerSimState = rollerMotor.getSimState();
    pivotSimState = pivotMotor.getSimState();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        coralSensor.periodic();


       // Logging
       // Logger.recordOutput("RollerVoltageOut", rollerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(loggerPath + "/PivotAngle", getPivotAngle());

        Logger.recordOutput(coralSensorLoggerPath + "/Coral intake at Goal", atGoal());
        Logger.recordOutput(coralSensorLoggerPath + "/IsDetected?", coralSensor.isDetected());

        Logger.recordOutput(rollerMotorLoggerPath + "/Velocity", rollerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(rollerMotorLoggerPath + "/Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(rollerMotorLoggerPath + "/Stator Current", rollerMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput(rollerMotorLoggerPath + "/Temp", rollerMotor.getDeviceTemp().getValueAsDouble() );

        Logger.recordOutput(pivotMotorLoggerPath + "/Voltage",pivotMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(pivotMotorLoggerPath + "/Pivot Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput(pivotMotorLoggerPath + "/Temp", pivotMotor.getDeviceTemp().getValueAsDouble());

        
        // SmartDashboard.putNumber(getName() + "/PivotMotor", pivotMotor.getMotorVoltage().getValueAsDouble());

        Logger.recordOutput("Intake/ L1 Mode", L1Mode);
        Logger.recordOutput("Intake/ Has Coral", hasCoral);

        if(intakeSpeed < -0.74){
          if(coralSensor.isDetected()){
            if(L1Mode){
              intakeSpeed = 0;
            }else{
              //intakeSpeed = CoralIntakeConstants.intakeSlow;
            }
          }
        }

        if(intakeDebouncer.calculate(coralSensor.isDetected())){
          hasCoral = true;
        }else{
          hasCoral = false;
        }
        
        setRollerOpenLoop(intakeSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run

  }

  public void holdCurrentPosition(){
    setAngle(targetPosition);
  }

  public boolean getL1Mode(){
    return L1Mode;
  }

  public boolean getHasCoral(){
    return hasCoral;
  }
  
  public boolean atGoal() {
    return Math.abs(targetPosition - getPivotAngle()) < CoralIntakeConstants.positionTolerance;
  }

  public double getPivotAngle(){
    return Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
  }

  public void setAngle(double angle){
    targetPosition = angle;
    posVoltage.withPosition(Units.degreesToRotations(angle)).withEnableFOC(true);
    pivotMotor.setControl(posVoltage);
  }

  public Command  setAngleCommand(double angle){
    return runOnce(() -> setAngle(angle));
  }

  //Controls the pivot and rollers 
  public void setRollerOpenLoop(double input){
    rollerOut.withOutput(input).withEnableFOC(true);
    rollerMotor.setControl(rollerOut);
  }

  public Command setRollerOpenLoopCommand (double input) {
    return Commands.runOnce(() -> setRollerOpenLoop(input));
  }


  public void setPivotOpenLoop(double input){
    pivotOut.withOutput(input);
    pivotMotor.setControl(pivotOut);
  }

  public Command setPivotOpenLoopCommand (double input) {
    return Commands.runOnce(() -> setPivotOpenLoop(input));
  }

  public Command setL1ModeON(){
    return new InstantCommand(() -> L1Mode = true);
  }

  public Command setL1ModeOFF(){
    return new InstantCommand(() -> L1Mode = false);
  }


  public void setIntake(){
    intakeSpeed = CoralIntakeConstants.intakeFast;
  }

  public void setEject(){
    intakeSpeed = CoralIntakeConstants.eject;
  }

  public void setEjectSlow(){
    intakeSpeed = CoralIntakeConstants.ejectSlow;
  }
  
  public void stopRoller(){
    intakeSpeed = 0;
  }
}

