package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PoseUtils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

public class DriveToPoseCommand extends Command {
  private final PIDController parallelController =
      DriveCommandConstants.makeTranslationController();

  private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

  private final PIDController perpendicularController = DriveCommandConstants.makeTranslationController();

  private final CommandSwerveDrivetrain drive;
  private final Supplier<Pose2d> targetPoseSupplier;
 

  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


  public DriveToPoseCommand(
      CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
   
  }

  

  @Override
  public void execute() {
    Pose2d robotPoseOriginal = drive.getState().Pose;
    Pose2d robotPose = robotPoseOriginal.plus(new Transform2d(0.0, Constants.robotArmCenterOffset, Rotation2d.kZero));
    Pose2d originalPose2d = targetPoseSupplier.get();
    Pose2d targetPose = originalPose2d.rotateAround(originalPose2d.getTranslation(), Rotation2d.kPi);

    Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

    double perpendicularError = PoseUtils.getPerpendicularError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);

    double parallelError = PoseUtils.getParallelError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/ParallelError", parallelError);

    double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();
    Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);

    double parallelSpeed = parallelController.calculate(-parallelError, 0);
    parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed : 0;

   double perpendicularSpeed = perpendicularController.calculate(-perpendicularError, 0);
    perpendicularSpeed = !perpendicularController.atSetpoint() ? perpendicularSpeed : 0;

    double angularSpeed = angleController.calculate(thetaError, 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            perpendicularSpeed,
            parallelSpeed,
            angularSpeed);

    drive.setControl(robotSpeeds.withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupt) {
    parallelController.reset();
    perpendicularController.reset();
    angleController.reset(0);
  }

    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return perpendicularController.atSetpoint()
    && parallelController.atSetpoint()
    && angleController.atSetpoint();
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () ->
            perpendicularController.atSetpoint()
                && parallelController.atSetpoint()
                && angleController.atSetpoint());
  }
}
