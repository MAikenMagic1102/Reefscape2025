// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Mechanism.SuperStructureMechanism;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator = new Elevator ();
  private final Arm arm = new Arm();

  private final SuperStructureMechanism mech = new SuperStructureMechanism();

  private enum scoreTarget{
    L1,
    L2,
    L3,
    L4,
    ALGAE
  };

  private scoreTarget currentTarget = scoreTarget.L1;

  private double armTargetAngle = 0.0;
  private double elevatorTargetHeight = 0.0;

  private boolean algaeNext = false;
  
  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    SmartDashboard.putString("Score Target", currentTarget.toString());
    
    // This method will be called once per scheduler run
    elevator.periodic();
    arm.periodic();
    mech.update(elevator.getPositionMeters());

    armTargetAngle = getScoreTargetArmAngle();
    elevatorTargetHeight = getScoreTargetElevatorPos();

    Logger.recordOutput("Superstructure/ Score Target", currentTarget.toString());
    Logger.recordOutput("Superstructure/ Elevator Target Height", elevatorTargetHeight);
    Logger.recordOutput("Superstructure/ Arm Target Angle", armTargetAngle);
    Logger.recordOutput("Superstructure/ Algae Next", algaeNext);

    SmartDashboard.putNumber("Elevator Target Height", elevatorTargetHeight);
    SmartDashboard.putNumber("Arm Target Angle", armTargetAngle);
    SmartDashboard.putBoolean("Algae NEXT", algaeNext);
  }

  public Command setAlgaeNext(){
    return new InstantCommand(() -> algaeNext = true);
  }

  public Command setAlgaeNextOFF(){
    return new InstantCommand(() -> algaeNext = false);
  }

  public boolean getAlgaeNext(){
    return algaeNext && currentTarget == scoreTarget.L4;
  }

  public boolean elevatorAtGoal(){
    return elevator.atGoal();
  }

  public Command setTargetL1(){
    return runOnce(() -> currentTarget = scoreTarget.L1);
  }

  public Command setTargetL2(){
    return runOnce(() -> currentTarget = scoreTarget.L2);
  }

  public Command setTargetL3(){
    return runOnce(() -> currentTarget = scoreTarget.L3);
  }

  public Command setTargetL4(){
    return runOnce(() -> currentTarget = scoreTarget.L4);
  }

  public Command setTargetAlgae(){
    return runOnce(() -> currentTarget = scoreTarget.ALGAE);
  }


  public double getScoreTargetElevatorPos(){
    double scoreTarget = 0.0;

    switch(currentTarget){
      case L1:
        scoreTarget = ElevatorConstants.reefL1;
      break;
      case L2:
        scoreTarget = ElevatorConstants.reefL2;
      break;
      case L3:
        scoreTarget = ElevatorConstants.reefL3;
      break;
      case L4:
        scoreTarget = ElevatorConstants.reefL4;
      break;
      case ALGAE:
      scoreTarget = ElevatorConstants.ALGAE;
      break;                  
    }
    
    return scoreTarget;
  }

  public Command setElevatorToScore(){
    return new InstantCommand(() -> elevator.setPositionMetersMM(elevatorTargetHeight));
  }

  public boolean isElevatorAtGoal(){
    return elevator.atGoal();
  }

  public boolean isElevatorSafe(){
    return elevator.aboveIntake();
  }

  public boolean isElevatorBelowHalf(){
    return elevator.belowHalf();
  }
  
  public boolean isArmSafeNeeded(){
    return currentTarget == scoreTarget.L4;
  }

  public Command setElevatorToScoreSafe(){
    return new InstantCommand(() -> elevator.setPositionMetersMM(ElevatorConstants.safePos));
  }

  public double getScoreTargetArmAngle(){
    double scoreTarget = 0.0;

    switch(currentTarget){
      case L1:
        scoreTarget = ArmConstants.reefL1;
      break;
      case L2:
        scoreTarget = ArmConstants.reefL2;
      break;
      case L3:
        scoreTarget = ArmConstants.reefL3;
      break;
      case L4:
        scoreTarget = ArmConstants.reefL4;
      break;      
      case ALGAE:
      scoreTarget = ArmConstants.ALGAE;
      break;                              
    }
    
    return scoreTarget;
  }

  public Command setArmToScore(){
    return new InstantCommand(() -> arm.setAnglePosition(armTargetAngle));
  }

  public boolean isArmHome(){
    return arm.armAtHome();
  }

  public boolean isArmAtGoal(){
    return arm.atGoal();
  }

  public boolean armHalfScored(){
    return arm.armHalfScored();
  }

  public boolean isL4Coral(){
    return currentTarget == scoreTarget.L4;
  }

  public Command setArmToHome() {
    return new InstantCommand(() -> arm.setAnglePosition(-1.5));
  }

  public Command runElevatorUp() {
    return new InstantCommand(() -> elevator.setOpenLoop(0.25));
  }
  public Command runElevatorDown(){
    return new InstantCommand(() -> elevator.setOpenLoop(-0.25));
  }

  public Command stopElevator() {
    return new InstantCommand(() -> elevator.setOpenLoop(0));
  }

  public Command setMiddlePos() {
     return new InstantCommand(() -> elevator.setPositionMetersMM(0.85));
 }

 public Command setHighPos(){
  return new InstantCommand(() -> elevator.setPositionMeters(1));
 }  

 public Command setElevatorHome(){
  return new InstantCommand(() -> elevator.setPositionMetersMM(0.009));
 }  

 public Command armUp() {
  return runOnce(() -> arm.setOpenLoop(0.4));
}

public Command armDown() {
  return runOnce(() -> arm.setOpenLoop(-0.4));
}

public Command armStop(){
  return runOnce(() -> arm.setOpenLoop(0.0));
}



}
