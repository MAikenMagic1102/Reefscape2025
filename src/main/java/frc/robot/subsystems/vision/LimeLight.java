// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// import frc.robot.subsystems.vision;
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimeLight extends SubsystemBase {
  /** Creates a new Limelight. */

  private double heightOfGoal = 57.13;
  private double heightOfRobotCamera = 10;
  private double cameraMountAngle = 30;

  private boolean hasNotAppliedPrio = false;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-magic");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry tx = limelightTable.getEntry("tx");


  public LimeLight() {
  }

  public double getLimelightDistance(){
    double distance = 0.0;

    double targetOffsetAngle_Vertical = Rotation2d.fromDegrees(cameraMountAngle + ty.getDouble(0.0)).getRadians();
    distance = (heightOfGoal-heightOfRobotCamera) / Math.tan(targetOffsetAngle_Vertical);

    return distance;

  }

  public double aimToTag(){
  
      double heading_error = -tx.getDouble(0.0);
      double steering_adjust = heading_error * Constants.Limelight.limelight_kP;

      return steering_adjust;
  }

  public boolean isAimedAtReef(){
    double m_tx = Math.abs(tx.getDouble(0.0));
    return m_tx > 0.0 && m_tx < 1.0;
  }

  public boolean seesAprilTag(){
    return LimelightHelpers.getTV("limelight-magic");
  }

  public boolean seesCoral(){
    return LimelightHelpers.getTV("limelight-coral");
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance To Goal", getLimelightDistance());
    SmartDashboard.putNumber("TX Value", tx.getDouble(0.0));
    SmartDashboard.putBoolean("Limelight Looky", LimelightHelpers.getTV("limelight-magic"));
    SmartDashboard.putBoolean("Limelight Sees Coral", seesCoral());

    try{
      if ((!hasNotAppliedPrio || DriverStation.isDisabled())) {
      if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        
        LimelightHelpers.setPriorityTagID("limelight-magic", 4);
        hasNotAppliedPrio = true;
      }

      if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
        LimelightHelpers.setPriorityTagID("limelight-magic", 7);
        hasNotAppliedPrio = true;
      }
      }

    }catch(Exception e){

    }

  }
}