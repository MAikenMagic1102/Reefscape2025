package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
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

    
    public AutoRoutine TetorisAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Tetoris Auto");
        final AutoTrajectory Tetoris = routine.trajectory("Tetoris");

        routine.active().onTrue(
            Tetoris.resetOdometry()
                .andThen(Tetoris.cmd())
        );
        return routine;
    }

    public AutoRoutine BermudaAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Bermuda Auto");
        final AutoTrajectory Bermuda = routine.trajectory("Bermuda");

        routine.active().onTrue(
            Bermuda.resetOdometry()
                .andThen(Bermuda.cmd())
        );
        return routine;
    }

    public AutoRoutine TetorisANDBermudaAuto() {
        final AutoRoutine routine = m_factory.newRoutine("TetorisANDBermuda Auto");
        final AutoTrajectory Tetoris = routine.trajectory("Tetoris");
        final AutoTrajectory Bermuda = routine.trajectory("Bermuda");

  
        routine.active().onTrue(
        Commands.sequence(
            Tetoris.resetOdometry(),
            Tetoris.cmd(),
            Bermuda.cmd()
            
            )
        );
        return routine;
    }
}
