// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralGripper;

import org.littletonrobotics.junction.Logger;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGripper extends SubsystemBase {
  private ThriftyNova grippers;

  private double coralSpeed = 0.0;

  private Timer delayTimer = new Timer();
  private Timer currentTimer = new Timer();

  private boolean hasCoral = false;

  /** Creates a new CoralGripper. */
  public CoralGripper() {
    grippers = new ThriftyNova(CoralGripperConstants.motorID, ThriftyNova.MotorType.MINION);

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
    // Clear errors here
    grippers.clearErrors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("CoralGripper Voltage", grippers.getVoltage());
    Logger.recordOutput("CoralGripper Stator Current", grippers.getStatorCurrent());

    SmartDashboard.putBoolean("Has Coral", hasCoral);

    if (coralSpeed > 0.49) {

      // Wait to scan current until after 0.25s to clear ramp up current spike
      if (delayTimer.hasElapsed(0.75)) {
          if (grippers.getStatorCurrent() < 25) {
              currentTimer.reset();
              currentTimer.stop();
          } else {
              currentTimer.start();
          }

          // Current spike of .25s reasonable to assume picked up game piece
          if (currentTimer.hasElapsed(0.25)) {
              coralSpeed = 0.0;
              hasCoral = true;
          }
      }
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
    return hasCoral;
  }

}
