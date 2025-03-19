package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.PoseUtils;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.PrepScore;
import frc.robot.commands.ReturnToHome;
import frc.robot.commands.ScoreCoral;
import frc.robot.field.FieldUtils;
import frc.robot.field.ReefPole;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.CoralGripper.CoralGripper;
import frc.robot.subsystems.Intake.CoralIntake;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain drive;
    private final CoralGripper m_coralGripper;
    private final CoralIntake m_coralIntake;
    private final Superstructure m_superstructure;

    public AutoRoutines(CommandSwerveDrivetrain drive, CoralGripper coralGripper, CoralIntake coralIntake, Superstructure superstructure) {

        this.drive = drive;
        m_factory = drive.createAutoFactory();
        m_coralGripper = coralGripper;
        m_coralIntake = coralIntake;
        m_superstructure = superstructure;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine LeftSideCross(){
        final AutoRoutine routine = m_factory.newRoutine("Left Sdie Cross+1");
         final AutoTrajectory LeftSideCross = routine.trajectory("Left Side Cross+1");

         routine.active().onTrue(
            Commands.sequence(
                LeftSideCross.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake),
                LeftSideCross.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
            )

         );
            return routine;
    }

    public AutoRoutine RightSideCross(){
        final AutoRoutine routine = m_factory.newRoutine("Right Sdie Cross+1");
         final AutoTrajectory RightSideCross = routine.trajectory("Right Side Cross+1");

         routine.active().onTrue(
            Commands.sequence(
                RightSideCross.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake),
                RightSideCross.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
            )
         );
            return routine;
    }
    public AutoRoutine MiddleCross(){
        final AutoRoutine routine = m_factory.newRoutine("Middle Cross+1");
         final AutoTrajectory MiddleCross = routine.trajectory("Middle Cross+1");

         routine.active().onTrue(
            Commands.sequence(
                MiddleCross.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake),
                MiddleCross.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                //Add PositionToPole
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
            )

         );
            return routine;
    }

    public Command positionToPole(Supplier<ReefPole> pole, double offsetMeters){
        return new DriveToPoseCommand(drive, 
        () -> PoseUtils.getPerpendicularOffsetPose(pole.get().getPose(), offsetMeters));
    }
    
   
}
