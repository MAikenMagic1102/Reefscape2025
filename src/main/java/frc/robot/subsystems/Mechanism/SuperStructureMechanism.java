// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanism;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/** Add your docs here. */
public class SuperStructureMechanism {

  private final Mechanism2d mech2d = new Mechanism2d(Units.inchesToMeters(2),Units.inchesToMeters(96));

  private final MechanismRoot2d elevatorRoot;
  private final MechanismLigament2d elevatorTower;
  private final MechanismLigament2d elevatorStage;  

   public SuperStructureMechanism(){
    elevatorRoot = mech2d.getRoot("Elevator Root", Units.inchesToMeters(1), Units.inchesToMeters(2));

    elevatorTower =
    elevatorRoot.append(new MechanismLigament2d("Elevator Tower", Units.inchesToMeters(32.42), 90, 12, new Color8Bit(Color.kOrange)));

    elevatorStage =
    elevatorTower.append(new MechanismLigament2d(
          "Elevator Stage",
          ElevatorConstants.elevatorStartingHeightMeters, 
          0,
          8,
          new Color8Bit(Color.kRed)));  

    SmartDashboard.putData("Elevator Sim", mech2d);
   }
   
   public void update(double elevatorHeightMeters){
        elevatorStage.setLength(elevatorHeightMeters);
   }
}
