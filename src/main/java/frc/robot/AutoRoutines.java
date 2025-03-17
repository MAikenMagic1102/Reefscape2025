package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.util.PoseUtils;

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
                //m_superstructure.setTargetL4(),
                //new IntakeDeploy(m_coralIntake),
                MiddleCross.cmd(),//.alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                //Add PositionToPole
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                new WaitCommand(1.5)
                //new ScoreCoral(m_coralGripper),
                //new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
            )

         );
            return routine;
    }

    public AutoRoutine TwoMeters(){
        final AutoRoutine routine = m_factory.newRoutine("TwoMeters");
        final AutoTrajectory TwoMeters = routine.trajectory("TwoMeters");

        routine.active().onTrue(
            TwoMeters.resetOdometry()
                .andThen(TwoMeters.cmd())
        );

        return routine;
    }

    public AutoRoutine ThreeMeters(){
        final AutoRoutine routine = m_factory.newRoutine("ThreeMeters");
        final AutoTrajectory ThreeMeters = routine.trajectory("ThreeMeters");

        routine.active().onTrue(
            ThreeMeters.resetOdometry()
                .andThen(ThreeMeters.cmd())
        );

        return routine;
    }


    public AutoRoutine FiveMeters(){
        final AutoRoutine routine = m_factory.newRoutine("FiveMeters");
        final AutoTrajectory FiveMeters = routine.trajectory("FiveMeters");

        routine.active().onTrue(
            FiveMeters.resetOdometry()
                .andThen(FiveMeters.cmd())
        );

        return routine;
    }


    public AutoRoutine RightToReef(){
        final AutoRoutine routine = m_factory.newRoutine("Right to Reef");
        final AutoTrajectory RightToReef1 = routine.trajectory("Right to Reef+1");
        final AutoTrajectory RightToReef2 = routine.trajectory("RHPS to Reef+2");
        final AutoTrajectory RightToReef3 = routine.trajectory("RHPS to Reef +3");
        final AutoTrajectory RightToReef4 = routine.trajectory("RHPS to Reef +4");
        final AutoTrajectory RightToReef5 = routine.trajectory("RHPS to Reef+5");
        final AutoTrajectory ReefToRHPS = routine.trajectory("Reef to RHPS");
        final AutoTrajectory RightToReef6 = routine.trajectory("RHPS to Reef+6");
        final AutoTrajectory ReefToRHPS2 = routine.trajectory("Reef to RHPS+2");
        routine.active().onTrue(
            Commands.sequence(
                RightToReef1.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake), 
                RightToReef1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    RightToReef2.cmd(),
                    new IntakeDeploy(m_coralIntake),
                    m_superstructure.setTargetL4(),
                        RightToReef3.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                        new WaitCommand(1.5),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            RightToReef4.cmd(),
                            new IntakeDeploy(m_coralIntake),
                            m_superstructure.setTargetL4(),
                                RightToReef5.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                new WaitCommand(1.5),
                                new ScoreCoral(m_coralGripper),
                                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    ReefToRHPS.cmd(),
                                    new IntakeDeploy(m_coralIntake),
                                    m_superstructure.setTargetL4(),
                                        RightToReef6.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                        new WaitCommand(1.5),
                                        new ScoreCoral(m_coralGripper),
                                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            ReefToRHPS2.cmd()
                     

            )


        );

        return routine;
    }

    public AutoRoutine LeftToReef(){
        final AutoRoutine routine = m_factory.newRoutine("Left to Reef");
        final AutoTrajectory LeftToReef1 = routine.trajectory("Left to Reef+1");
        final AutoTrajectory LeftToReef2 = routine.trajectory("LHPS to Reef+2");
        final AutoTrajectory LeftToReef3 = routine.trajectory("LHPS to Reef+3");
        final AutoTrajectory LeftToReef4 = routine.trajectory("LHPS to Reef+4");
        final AutoTrajectory LeftToReef5 = routine.trajectory("LHPS to Reef+5");
        final AutoTrajectory ReefToLHPS = routine.trajectory("Reef to LHPS");
        final AutoTrajectory LeftToReef6 = routine.trajectory("LHPS to Reef+6");
        final AutoTrajectory ReefToLHPS2 = routine.trajectory("Reef to LHPS+2");

        routine.active().onTrue(
            Commands.sequence(
                LeftToReef1.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake), 
                LeftToReef1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    LeftToReef2.cmd(),
                    new IntakeDeploy(m_coralIntake),
                    m_superstructure.setTargetL4(),
                        LeftToReef3.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                        new WaitCommand(1.5),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            LeftToReef4.cmd(),
                            new IntakeDeploy(m_coralIntake),
                            m_superstructure.setTargetL4(),
                                LeftToReef5.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                new WaitCommand(1.5),
                                new ScoreCoral(m_coralGripper),
                                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    ReefToLHPS.cmd(),
                                    new IntakeDeploy(m_coralIntake),
                                    m_superstructure.setTargetL4(),
                                        LeftToReef6.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                        new WaitCommand(1.5),
                                        new ScoreCoral(m_coralGripper),
                                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            ReefToLHPS2.cmd()
                 
                     

            )


        );

        return routine;
    }

    public AutoRoutine LeftToPush(){
        final AutoRoutine routine = m_factory.newRoutine("Left to Push");
        final AutoTrajectory LeftToReef1 = routine.trajectory("Left to Push");
        final AutoTrajectory LeftToReef2 = routine.trajectory("LHPS to Reef+2");
        final AutoTrajectory LeftToReef3 = routine.trajectory("LHPS to Reef+3");
        final AutoTrajectory LeftToReef4 = routine.trajectory("LHPS to Reef+4");
        final AutoTrajectory LeftToReef5 = routine.trajectory("LHPS to Reef+5");
        final AutoTrajectory ReefToLHPS = routine.trajectory("Reef to LHPS");
        final AutoTrajectory LeftToReef6 = routine.trajectory("LHPS to Reef+6");
        final AutoTrajectory ReefToLHPS2 = routine.trajectory("Reef to LHPS+2");

        routine.active().onTrue(
            Commands.sequence(
                LeftToReef1.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake), 
                LeftToReef1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    LeftToReef2.cmd(),
                    new IntakeDeploy(m_coralIntake),
                    m_superstructure.setTargetL4(),
                        LeftToReef3.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                        new WaitCommand(1.5),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            LeftToReef4.cmd(),
                            new IntakeDeploy(m_coralIntake),
                            m_superstructure.setTargetL4(),
                                LeftToReef5.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                new WaitCommand(1.5),
                                new ScoreCoral(m_coralGripper),
                                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    ReefToLHPS.cmd(),
                                    new IntakeDeploy(m_coralIntake),
                                    m_superstructure.setTargetL4(),
                                        LeftToReef6.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                        new WaitCommand(1.5),
                                        new ScoreCoral(m_coralGripper),
                                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            ReefToLHPS2.cmd()
                 
                     

            )


        );

        return routine;
    }

    public AutoRoutine RightToPush(){
        final AutoRoutine routine = m_factory.newRoutine("Right to Push");
        final AutoTrajectory RightToReef1 = routine.trajectory("Right to Push");
        final AutoTrajectory RightToReef2 = routine.trajectory("RHPS to Reef+2");
        final AutoTrajectory RightToReef3 = routine.trajectory("RHPS to Reef +3");
        final AutoTrajectory RightToReef4 = routine.trajectory("RHPS to Reef +4");
        final AutoTrajectory RightToReef5 = routine.trajectory("RHPS to Reef+5");
        final AutoTrajectory ReefToRHPS = routine.trajectory("Reef to RHPS");
        final AutoTrajectory RightToReef6 = routine.trajectory("RHPS to Reef+6");
        final AutoTrajectory ReefToRHPS2 = routine.trajectory("Reef to RHPS+2");
        routine.active().onTrue(
            Commands.sequence(
                RightToReef1.resetOdometry(),
                // m_superstructure.setTargetL4(),
                // new IntakeDeploy(m_coralIntake), 
                RightToReef1.cmd(),
                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                // new WaitCommand(1.5),
                // new ScoreCoral(m_coralGripper),
                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    RightToReef2.cmd(),
                    // new IntakeDeploy(m_coralIntake),
                    // m_superstructure.setTargetL4(),
                        RightToReef3.cmd(),
                        // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                        // new WaitCommand(1.5),
                        // new ScoreCoral(m_coralGripper),
                        // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            RightToReef4.cmd(),
                            // new IntakeDeploy(m_coralIntake),
                            // m_superstructure.setTargetL4(),
                                RightToReef5.cmd(),
                                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                // new WaitCommand(1.5),
                                // new ScoreCoral(m_coralGripper),
                                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    ReefToRHPS.cmd(),
                                    // new IntakeDeploy(m_coralIntake),
                                    // m_superstructure.setTargetL4(),
                                        RightToReef6.cmd(),
                                        // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                        // new WaitCommand(1.5),
                                        // new ScoreCoral(m_coralGripper),
                                        // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            ReefToRHPS2.cmd()
                     

            )


        );

        return routine;
    }

    public AutoRoutine LeftToOne(){
        final AutoRoutine routine = m_factory.newRoutine("Left to One");
        final AutoTrajectory LeftToReef1 = routine.trajectory("Left to Reef+1");

        routine.active().onTrue(
            Commands.sequence(
                LeftToReef1.resetOdometry(),
                // m_superstructure.setTargetL4(),
                // new IntakeDeploy(m_coralIntake), 
                LeftToReef1.cmd(),
                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5)
                // new WaitCommand(1.5),
                // new ScoreCoral(m_coralGripper),
                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
                
         ) );

            return routine;
    }


    public Command positionToPole(Supplier<ReefPole> pole, double offsetMeters){
        return new DriveToPoseCommand(drive, 
        () -> PoseUtils.getPerpendicularOffsetPose(pole.get().getPose(), offsetMeters));
    }
    
   
}
