// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralGripper;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGripper extends SubsystemBase {
  private ThriftyNova grippers;
  private CANrange coralDetect;

  private double coralSpeed = 0.0;

  private Timer delayTimer = new Timer();
  private Timer currentTimer = new Timer();

  private boolean hasCoral = false;

  /** Creates a new CoralGripper. */
  public CoralGripper() {
    grippers = new ThriftyNova(CoralGripperConstants.motorID, ThriftyNova.MotorType.MINION);
    coralDetect = new CANrange(33, "rio");

    grippers.factoryReset();

    grippers.setMotorType(MotorType.MINION);
    grippers.setBrakeMode(true);
    grippers.setInverted(true);
    grippers.setMaxCurrent(CurrentType.SUPPLY, 35);
    grippers.setTempThrottleEnable(false);
    grippers.setNTLogging(false);
    
    // Iterate through errors and check them
    for (var err : grippers.getErrors()) {
    // The user can handle the errors
      System.out.println("Error " + err.toString());
    }

    CANrangeConfiguration cfg = new CANrangeConfiguration();
    cfg.ToFParams.withUpdateMode(UpdateModeValue.ShortRangeUserFreq);
    cfg.FovParams.withFOVRangeX(27);
    cfg.FovParams.withFOVRangeY(27);
    cfg.ProximityParams.withProximityThreshold(0.075);
    coralDetect.getConfigurator().apply(cfg);

    // Clear errors here
    grippers.clearErrors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("CoralGripper Voltage", grippers.getVoltage());
    Logger.recordOutput("CoralGripper Stator Current", grippers.getStatorCurrent());

    Logger.recordOutput("CoralDetect", coralDetect.getIsDetected().getValue());

  //   if (coralSpeed > 0.49) {

  //     // Wait to scan current until after 0.25s to clear ramp up current spike
  //     // if (delayTimer.hasElapsed(0.75)) {
  //     //     if (grippers.getStatorCurrent() < 25) {
  //     //         currentTimer.reset();
  //     //         currentTimer.stop();
  //     //     } else {
  //     //         currentTimer.start();
  //     //     }

  //     //     // Current spike of .25s reasonable to assume picked up game piece
  //     //     if (currentTimer.hasElapsed(0.25)) {
  //     //         coralSpeed = 0.0;
  //     //         hasCoral = true;
  //     //     }
  //     // }
  // }

    if(hasCoral() && coralSpeed == 0.65){
      coralSpeed = 0.03;
    }

    grippers.setPercent(coralSpeed);

  }

  public void setRollerOpenLoop(double input){
    grippers.setPercent(input);
  }

  public Command setRollerOpenLoopCommand(double input){
    return runOnce(() -> setRollerOpenLoop(input));
  }

  public void setIntake(){
    coralSpeed = CoralGripperConstants.intake;
    delayTimer.start();
  }

  public void setEject(){
    coralSpeed = CoralGripperConstants.eject;
    hasCoral = false;
  }

  public void setStop(){
    coralSpeed = 0;
  }

  public boolean hasCoral(){
    return coralDetect.getIsDetected().getValue();
  }

}
