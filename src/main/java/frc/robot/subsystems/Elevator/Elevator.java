// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
          false,
          ElevatorConstants.elevatorStartingHeightMeters,
          0.005, 0.0);

  private DutyCycleOut dutyCycleOutput = new DutyCycleOut(0);

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    ElevatorSim.update(0.02);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(ElevatorSim.getCurrentDrawAmps()));

    motorSim.addRotorPosition(ElevatorSim.getPositionMeters()/(ElevatorConstants.elevatorPullyCircum / ElevatorConstants.elevatorGearing));
  }

  public boolean atGoal() {
    return false;
  }

  public double getPositionMeters() {
    return motorL.getPosition().getValueAsDouble()*ElevatorConstants.elevatorPullyCircum;
  }

  public void setPositionMeters() {

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

}
