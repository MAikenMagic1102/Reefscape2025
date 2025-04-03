package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoRunToCoral;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.IntakeRollersOn;
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
        final AutoRoutine routine = m_factory.newRoutine("SimplePath");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            Commands.sequence(
                new IntakeDeploy(m_coralIntake),
                new IntakeRollersOn(m_coralIntake, m_coralGripper),
                new AutoRunToCoral(drive).withDeadline(new WaitUntilCommand(m_coralIntake::getHasCoral).withTimeout(3.0)),
                new WaitUntilCommand(m_coralGripper::hasCoral).withTimeout(1.0),
                
                //m_superstructure.setTargetL3().andThen(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                new InstantCommand(() -> m_coralIntake.stopRoller())
            )
        );

        return routine;
    }

    public AutoRoutine LeftSideCross(){
        final AutoRoutine routine = m_factory.newRoutine("Left Sdie Cross+1");
         final AutoTrajectory LeftSideCross = routine.trajectory("Left Side Cross");

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
         final AutoTrajectory RightSideCross = routine.trajectory("Right Side Cross");

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
        final AutoRoutine routine = m_factory.newRoutine("Middle Cross");
         final AutoTrajectory MiddleCross = routine.trajectory("Middle Cross+1");

         routine.active().onTrue(
            Commands.sequence(
                //MiddleCross.resetOdometry(),
                m_superstructure.setTargetL3(),
                new IntakeDeploy(m_coralIntake), 
                MiddleCross.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                m_superstructure.setTargetL4(),
                new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                new WaitCommand(0.25),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, Constants.robotToReefOffset),
                new WaitCommand(0.25),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
            )

         );
            return routine;
    }

    public AutoRoutine TwoMeters(){
        final AutoRoutine routine = m_factory.newRoutine("TwoMeters");
        final AutoTrajectory TwoMeters = routine.trajectory("TwoMeters");

        routine.active().onTrue(
            Commands.sequence(
                TwoMeters.resetOdometry(),
                new IntakeRetract(m_coralIntake),
                TwoMeters.cmd()
            )
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
        //"RR" Right reef, robot is at the reef going to RHPS
        //"RHPSR" Right human player station, robot is at the HPS going to the reef
        final AutoRoutine routine = m_factory.newRoutine("Right to Reef");
        final AutoTrajectory RR1 = routine.trajectory("RR+1");
        final AutoTrajectory RR2 = routine.trajectory("RR+2");
        final AutoTrajectory RHPSR3 = routine.trajectory("RHPSR+3");
        final AutoTrajectory RR4 = routine.trajectory("RR+4");
        final AutoTrajectory RHPSR5 = routine.trajectory("RHPSR+5");
        final AutoTrajectory RR6 = routine.trajectory("RR+6");
        final AutoTrajectory RHPSR7 = routine.trajectory("RHPSR+7");
        final AutoTrajectory RR8 = routine.trajectory("RR+8");
        routine.active().onTrue(
            Commands.sequence(
                RR1.resetOdometry(),
                // m_superstructure.setTargetL4(),
                // new IntakeDeploy(m_coralIntake), 
                RR1.cmd(),
                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                // new WaitCommand(1.5),
                // new ScoreCoral(m_coralGripper),
                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    RR2.cmd(),
                    // new IntakeDeploy(m_coralIntake),
                    // m_superstructure.setTargetL4(),
                        RHPSR3.cmd(),
                        // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                        // new WaitCommand(1.5),
                        // new ScoreCoral(m_coralGripper),
                        // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            RR4.cmd(),
                            // new IntakeDeploy(m_coralIntake),
                            // m_superstructure.setTargetL4(),
                                RHPSR5.cmd(),
                                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                // new WaitCommand(1.5),
                                // new ScoreCoral(m_coralGripper),
                                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    RR6.cmd(),
                                    // new IntakeDeploy(m_coralIntake),
                                    // m_superstructure.setTargetL4(),
                                        RHPSR7.cmd(),
                                        // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                        // new WaitCommand(1.5),
                                        // new ScoreCoral(m_coralGripper),
                                        // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            RR8.cmd()
                     

            )


        );

        return routine;
    }

    public AutoRoutine LeftToReef(){
        final AutoRoutine routine = m_factory.newRoutine("Left to Reef");
        final AutoTrajectory LR1 = routine.trajectory("LR+1");
        final AutoTrajectory LR2 = routine.trajectory("LR+2");
        final AutoTrajectory LHPSR3 = routine.trajectory("LHPSR+3");
        final AutoTrajectory LR4 = routine.trajectory("LR+4");
        final AutoTrajectory LHPSR5 = routine.trajectory("LHPSR+5");
        final AutoTrajectory LR6 = routine.trajectory("LR+6");
        final AutoTrajectory LHPSR7 = routine.trajectory("LHPSR+7");
        final AutoTrajectory LR8 = routine.trajectory("LR+8");

        routine.active().onTrue(
            Commands.sequence(
                LR1.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake), 
                LR1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    LR2.cmd(),
                    new IntakeDeploy(m_coralIntake),
                    m_superstructure.setTargetL4(),
                        LHPSR3.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                        new WaitCommand(1.5),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            LR4.cmd(),
                            new IntakeDeploy(m_coralIntake),
                            m_superstructure.setTargetL4(),
                                LHPSR5.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                new WaitCommand(1.5),
                                new ScoreCoral(m_coralGripper),
                                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    LR6.cmd(),
                                    new IntakeDeploy(m_coralIntake),
                                    m_superstructure.setTargetL4(),
                                        LHPSR7.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                        new WaitCommand(1.5),
                                        new ScoreCoral(m_coralGripper),
                                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            LR8.cmd()
                 
                     

            )


        );

        return routine;
    }

    public AutoRoutine LeftToPush(){
        final AutoRoutine routine = m_factory.newRoutine("Left to Push");
        final AutoTrajectory LeftPush = routine.trajectory("Left Push");
        final AutoTrajectory LR2 = routine.trajectory("LR+2");
        final AutoTrajectory LHPSR3 = routine.trajectory("LHPSR+3");
        final AutoTrajectory LR4 = routine.trajectory("LR+4");
        final AutoTrajectory LHPSR5 = routine.trajectory("LHPSR+5");
        final AutoTrajectory LR6 = routine.trajectory("LR+6");
        final AutoTrajectory LHPSR7 = routine.trajectory("LHPSR+7");
        final AutoTrajectory LR8 = routine.trajectory("LR+8");


        routine.active().onTrue(
            Commands.sequence(
                LeftPush.resetOdometry(),
                m_superstructure.setTargetL4(),
                new IntakeDeploy(m_coralIntake), 
                LeftPush.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                new WaitCommand(1.5),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    LR2.cmd(),
                    new IntakeDeploy(m_coralIntake),
                    m_superstructure.setTargetL4(),
                        LHPSR3.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                        new WaitCommand(1.5),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            LR4.cmd(),
                            new IntakeDeploy(m_coralIntake),
                            m_superstructure.setTargetL4(),
                                LHPSR5.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                new WaitCommand(1.5),
                                new ScoreCoral(m_coralGripper),
                                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    LR6.cmd(),
                                    new IntakeDeploy(m_coralIntake),
                                    m_superstructure.setTargetL4(),
                                        LHPSR7.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                        new WaitCommand(1.5),
                                        new ScoreCoral(m_coralGripper),
                                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            LR8.cmd()
                 
                     

            )


        );

        return routine;
    }

    public AutoRoutine RightToPush(){
        final AutoRoutine routine = m_factory.newRoutine("Right to Push");
        final AutoTrajectory RightPush = routine.trajectory("Right Push");
        final AutoTrajectory RR2 = routine.trajectory("RR+2");
        final AutoTrajectory RHPSR3 = routine.trajectory("RHPSR+3");
        final AutoTrajectory RR4 = routine.trajectory("RR+4");
        final AutoTrajectory RHPSR5 = routine.trajectory("RHPSR+5");
        final AutoTrajectory RR6 = routine.trajectory("RR+6");
        final AutoTrajectory RHPSR7 = routine.trajectory("RHPSR+7");
        final AutoTrajectory RR8 = routine.trajectory("RR+8");
        routine.active().onTrue(
            Commands.sequence(
                RightPush.resetOdometry(),
                // m_superstructure.setTargetL4(),
                // new IntakeDeploy(m_coralIntake), 
                RightPush.cmd(),
                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                // new WaitCommand(1.5),
                // new ScoreCoral(m_coralGripper),
                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                    RR2.cmd(),
                    // new IntakeDeploy(m_coralIntake),
                    // m_superstructure.setTargetL4(),
                        RHPSR3.cmd(),
                        // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                        // new WaitCommand(1.5),
                        // new ScoreCoral(m_coralGripper),
                        // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                            RR4.cmd(),
                            // new IntakeDeploy(m_coralIntake),
                            // m_superstructure.setTargetL4(),
                                RHPSR5.cmd(),
                                // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                positionToPole(() -> FieldUtils.getClosestReef().leftPole, 0.5),
                                // new WaitCommand(1.5),
                                // new ScoreCoral(m_coralGripper),
                                // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                    RR6.cmd(),
                                    // new IntakeDeploy(m_coralIntake),
                                    // m_superstructure.setTargetL4(),
                                        RHPSR7.cmd(),
                                        // .alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                                        positionToPole(() -> FieldUtils.getClosestReef().rightPole, 0.5),
                                        // new WaitCommand(1.5),
                                        // new ScoreCoral(m_coralGripper),
                                        // new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper),
                                            RR8.cmd()
                     

            )


        );

        return routine;
    }

    public AutoRoutine LeftToOne(){
        final AutoRoutine routine = m_factory.newRoutine("Left to One");
        final AutoTrajectory LR1 = routine.trajectory("LR+1");

        routine.active().onTrue(
            Commands.sequence(
                //LeftToReef1.resetOdometry(),
                m_superstructure.setTargetL3(),
                new IntakeDeploy(m_coralIntake), 
                LR1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                m_superstructure.setTargetL4(),
                new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                new WaitCommand(0.25),
                positionToPole(() -> FieldUtils.getClosestReef().rightPole, Constants.robotToReefOffset),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
                
         ) );

            return routine;
    }

    public AutoRoutine LeftToOnePlus(){
        final AutoRoutine routine = m_factory.newRoutine("Left to One Plus");
        final AutoTrajectory LR1 = routine.trajectory("LR+1");
        final AutoTrajectory LR2 = routine.trajectory("LR+2");
        final AutoTrajectory LHPSR3 = routine.trajectory("LHPSR+3");
        final AutoTrajectory RetryL = routine.trajectory("RetryL");
        final AutoTrajectory LR4 = routine.trajectory("LR+4");
        final AutoTrajectory LHPSR5 = routine.trajectory("LHPSR+5");

        routine.active().onTrue(
            Commands.sequence(
                //LeftToReef1.resetOdometry(),
                m_superstructure.setTargetL3(),
                new IntakeDeploy(m_coralIntake), 
                LR1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                m_superstructure.setTargetL4(),
                new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                new WaitCommand(0.25),
                positionToPole(() -> FieldUtils.getClosestReef().leftPole, Constants.robotToReefOffset),
                new WaitCommand(0.1),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper).alongWith(
                    Commands.sequence(
                       new WaitUntilCommand(m_superstructure::isElevatorBelowHalf),
                       new IntakeRollersOn(m_coralIntake, m_coralGripper),
                       LR2.cmd()
                    )
                ),
                
                new WaitCommand(0.1),

                new InstantCommand(() -> m_coralIntake.stopRoller()),

                new ConditionalCommand(
                //Has Coral
                Commands.sequence(
                    m_superstructure.setTargetL3().andThen(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                    LHPSR3.cmd(),
                    m_superstructure.setTargetL4(),
                    new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                    new WaitCommand(0.25),
                    positionToPole(() -> FieldUtils.getClosestReef().rightPole, Constants.robotToReefOffset),
                    new WaitCommand(0.1),
                    new ScoreCoral(m_coralGripper),
                    new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper).alongWith(
                        Commands.sequence(
                           new WaitUntilCommand(m_superstructure::isElevatorBelowHalf),
                           new IntakeRollersOn(m_coralIntake, m_coralGripper),
                           LR4.cmd()
                        )
                    )

                ),

                //Does Not Have Coral
                new ParallelRaceGroup(
                new WaitUntilCommand(m_coralGripper::hasCoral),
                Commands.sequence(
                    new IntakeRollersOn(m_coralIntake, m_coralGripper),
                    RetryL.cmd(),
                    new WaitCommand(0.2)
                )), 

                m_coralGripper::hasCoral),

                new WaitCommand(0.1),
                new InstantCommand(() -> m_coralIntake.stopRoller()),

                new ConditionalCommand(
                    //Has Coral
                    Commands.sequence(
                        m_superstructure.setTargetL3().andThen(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        LHPSR5.cmd(),
                        m_superstructure.setTargetL4(),
                        new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                        new WaitCommand(0.25),
                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, Constants.robotToReefOffset),
                        new WaitCommand(0.1),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
                    ),
    
                    //Does Not Have Coral
                    new ParallelRaceGroup(
                    new WaitUntilCommand(m_coralGripper::hasCoral),
                    Commands.sequence(
                        new IntakeRollersOn(m_coralIntake, m_coralGripper),
                        RetryL.cmd(),
                        new WaitCommand(0.2)
                    )), 
    
                    m_coralGripper::hasCoral),

                    new InstantCommand(() -> m_coralIntake.stopRoller())

         ) );

            return routine;
    }

    public AutoRoutine RightToOnePlus(){
        final AutoRoutine routine = m_factory.newRoutine("Right to One Plus");
        final AutoTrajectory RR1 = routine.trajectory("RR+1");
        final AutoTrajectory RR2 = routine.trajectory("RR+2");
        final AutoTrajectory RHPSR3 = routine.trajectory("RHPSR+3");
        final AutoTrajectory RetryR = routine.trajectory("RetryR");
        final AutoTrajectory RR4 = routine.trajectory("RR+4");
        final AutoTrajectory RHPSR5 = routine.trajectory("RHPSR+5");

        routine.active().onTrue(
            Commands.sequence(
                //LeftToReef1.resetOdometry(),
                m_superstructure.setTargetL3(),
                new IntakeDeploy(m_coralIntake), 
                RR1.cmd().alongWith(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                m_superstructure.setTargetL4(),
                new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                new WaitCommand(0.25),
                positionToPole(() -> FieldUtils.getClosestReef().leftPole, Constants.robotToReefOffset),
                new WaitCommand(0.1),
                new ScoreCoral(m_coralGripper),
                new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper).alongWith(
                    Commands.sequence(
                       new WaitUntilCommand(m_superstructure::isElevatorBelowHalf),
                       new IntakeRollersOn(m_coralIntake, m_coralGripper),
                       RR2.cmd()
                    )
                ),
                
                new WaitCommand(0.1),

                new InstantCommand(() -> m_coralIntake.stopRoller()),

                new ConditionalCommand(
                //Has Coral
                Commands.sequence(
                    m_superstructure.setTargetL3().andThen(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                    RHPSR3.cmd(),
                    m_superstructure.setTargetL4(),
                    new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                    new WaitCommand(0.25),
                    positionToPole(() -> FieldUtils.getClosestReef().rightPole, Constants.robotToReefOffset),
                    new WaitCommand(0.1),
                    new ScoreCoral(m_coralGripper),
                    new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper).alongWith(
                        Commands.sequence(
                           new WaitUntilCommand(m_superstructure::isElevatorBelowHalf),
                           new IntakeRollersOn(m_coralIntake, m_coralGripper),
                           RR4.cmd()
                        )
                    )

                ),

                //Does Not Have Coral
                new ParallelRaceGroup(
                new WaitUntilCommand(m_coralGripper::hasCoral),
                Commands.sequence(
                    new IntakeRollersOn(m_coralIntake, m_coralGripper),
                    RetryR.cmd(),
                    new WaitCommand(0.2)
                )), 

                m_coralGripper::hasCoral),

                new WaitCommand(0.1),
                new InstantCommand(() -> m_coralIntake.stopRoller()),

                new ConditionalCommand(
                    //Has Coral
                    Commands.sequence(
                        m_superstructure.setTargetL3().andThen(new PrepScore(m_superstructure, m_coralGripper, m_coralIntake)),
                        RHPSR5.cmd(),
                        m_superstructure.setTargetL4(),
                        new PrepScore(m_superstructure, m_coralGripper, m_coralIntake),
                        new WaitCommand(0.25),
                        positionToPole(() -> FieldUtils.getClosestReef().leftPole, Constants.robotToReefOffset),
                        new WaitCommand(0.1),
                        new ScoreCoral(m_coralGripper),
                        new ReturnToHome(m_superstructure, m_coralIntake, m_coralGripper)
                    ),
    
                    //Does Not Have Coral
                    new ParallelRaceGroup(
                    new WaitUntilCommand(m_coralGripper::hasCoral),
                    Commands.sequence(
                        new IntakeRollersOn(m_coralIntake, m_coralGripper),
                        RetryR.cmd(),
                        new WaitCommand(0.2)
                    )), 
    
                    m_coralGripper::hasCoral),

                    new InstantCommand(() -> m_coralIntake.stopRoller())

         ) );

            return routine;
    }

    public AutoRoutine WTFISBROONABTOT(){
        final AutoRoutine routine = m_factory.newRoutine("WHYYYY");
        final AutoTrajectory LR2 = routine.trajectory("LR+2");

        routine.active().onTrue(
            LR2.resetOdometry().andThen(LR2.cmd())
            
        );
        

        return routine; 
    }


    public Command positionToPole(Supplier<ReefPole> pole, double offsetMeters){
        return new DriveToPoseCommand(drive, 
        () -> PoseUtils.getPerpendicularOffsetPose(pole.get().getPose(), offsetMeters)).withTimeout(1.0);
    }
    
   
}


