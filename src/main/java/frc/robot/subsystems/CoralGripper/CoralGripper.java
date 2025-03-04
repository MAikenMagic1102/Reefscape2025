// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralGripper;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGripper extends SubsystemBase {
  private ThriftyNova grippers;

  /** Creates a new CoralGripper. */
  public CoralGripper() {
    grippers = new ThriftyNova(CoralGripperConstants.motorID, ThriftyNova.MotorType.MINION);

    grippers.factoryReset();

    grippers.setMotorType(MotorType.MINION);
    grippers.setBrakeMode(true);
    grippers.setInverted(true);
    grippers.setMaxCurrent(CurrentType.STATOR, 20);
    grippers.setMaxCurrent(CurrentType.SUPPLY, 35);
    
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
  }

  public void setRollerOpenLoop(double input){
    grippers.setPercent(input);
  }

  public Command setRollerOpenLoopCommand(double input){
    return runOnce(() -> setRollerOpenLoop(input));
  }
}
