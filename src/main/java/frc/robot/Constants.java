package frc.robot;

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


}
