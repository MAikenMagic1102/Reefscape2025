// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.CoralIntakeConstants;

public class CoralIntake extends SubsystemBase {
 //The motor that controls rollers on the coral intake
  private TalonFX rollerMotor;
  // The motor that controls the Pivot motor on the coral intake
  private TalonFX pivotMotor;

  private TalonFXSimState rollerSimState;
  private TalonFXSimState intakeSimState;
  private DCMotor intakeGearbox = DCMotor.getKrakenX60Foc(30);

  private DCMotorSim rollerSim =  new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeGearbox, 0.001, 4.0), intakeGearbox);



  // private Sim CoralIntakeSim = 
  // new CoralIntakeSim(
  //       cIntakeGearbox,
  //       CoralIntakeConstants.cIntakeGearing,
  //       CoralIntakeConstants.cIntakeMass,
  //       CoralIntakeConstants.cIntakeMinHeightMeters,
  //       CoralIntakeConstants.cIntakeMaxHeightMeters,
  //       false,
  //       CoralIntakeConstants.cIntakeStartingHeightMeters,
  //       0.005, 0.0);

  private DutyCycleOut rollerOut = new DutyCycleOut(0);
  private DutyCycleOut pivotOut = new DutyCycleOut(0);
  
  
  /** Creates a new CoralIntake. */
  public CoralIntake() {
    rollerMotor = new TalonFX(CoralIntakeConstants.rollerMotorID);
    pivotMotor = new TalonFX(CoralIntakeConstants.pivotMotorID);

    rollerSimState = rollerMotor.getSimState();

  }

   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putNumber(getName() + "/RollerVoltageOut", rollerMotor.getMotorVoltage().getValueAsDouble());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());
    

  }
  

final boolean atGoal() {
  return false;
}

public double getAngle(){
  return 0.0;
}

public void setAngle(){

}

//Controls the pivot and rollers 
public void setOpenLoop(double input){
  // DutyCycleOut dutyCycleOutput;
  //       dutyCycleOutput.withOutput(input);
  rollerOut.withOutput(input);

  rollerMotor.setControl(rollerOut);

  pivotOut.withOutput(input);

  pivotMotor.setControl(pivotOut);

  // pivotMotor.setControl(new DutyCycleOut(input));

}

public Command setOpenLoopCommand (double input) {
  return Commands.runEnd(() -> this.setOpenLoop(input), () -> this.setOpenLoopCommand(0), this);
  // return new RunCommand(() -> this.setOpenLoopCommand(input))
}


public void setHome(){}



public Command setRollerOpenLoop(double outPut) {
  return runOnce(() -> setOpenLoop(outPut));
}

public Command setPivotOPenLoop(double outPut) {
 return runOnce(() -> setOpenLoop(outPut));
}

}



