package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoRunToCoral extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    double tx = 0;
    double omegaSpeed = 0;

    boolean m_isFinished = false;

    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(.1, .01);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(.1, 0, 0, OMEGA_CONSTRATINTS);

    /** Creates a new aimAtNote. */
    public AutoRunToCoral(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        omegaController.setTolerance(1);
        omegaController.setGoal(0);
        addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        omegaController.reset(tx);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tx = LimelightHelpers.getTX("limelight");
        omegaSpeed = omegaController.calculate(tx);
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        if (LimelightHelpers.getTV("limelight")) {
            m_drivetrain.setControl(m_forwardStraight
                    .withVelocityX(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * (1 - Math.abs(tx) / 32) * .4) // Constants.halfSpeed
                    .withVelocityY(0)
                    .withRotationalRate(omegaSpeed * -1));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       omegaController.reset(0);
        m_drivetrain.setControl(m_forwardStraight
            .withVelocityX(0) // Constants.halfSpeed
            .withVelocityY(0)
            .withRotationalRate(0));        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}