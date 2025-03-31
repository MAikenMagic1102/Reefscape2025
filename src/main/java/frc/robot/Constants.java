package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
    
        /** Running a physics simulator. */
        SIM,
    
        /** Replaying from a log file. */
        REPLAY
      }
      public static final Mode simMode = Mode.SIM;

      public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

      public static final double triggerPressedThreshold = 0.1;

      public static final double controllerDeadband = 0.15;

      public static final String canivore = "can2";
      public static double robotArmCenterOffset = Units.inchesToMeters(1.75);

 
      public static final double robotToReefOffset = 0.62;





      public static final class Limelight{
        public static final double limelight_kP = 0.0275;
        public static final double min_command = 0.05;

            public static class FieldConstants{
        public static final Pose2d BLUE_Reef = new Pose2d(Units.inchesToMeters(-1.5+12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_Reef = new Pose2d(Units.inchesToMeters(652.73-12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final double BLUE_AUTO_PENALTY_LINE = 8.6; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 8; // X distance from origin to center of the robot almost fully crossing the midline

        public static final double Coral_DIAMETER = 14; // Outer diameter of note

    }

  

    }

}
