// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorTest;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.L4finalPos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralGripper.CoralGripper;
import frc.robot.subsystems.Intake.CoralIntake;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController programmerJoystick = new CommandXboxController(2);
    private final CommandXboxController operatorJoy = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Superstructure superstructure = new Superstructure();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private final CoralIntake coralIntake = new CoralIntake();
    private final CoralGripper coralGripper = new CoralGripper();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        operatorJoy.povUp().whileTrue(superstructure.runElevatorUp()).onFalse(superstructure.stopElevator());
        operatorJoy.povDown().whileTrue(superstructure.runElevatorDown()).onFalse(superstructure.stopElevator());
        programmerJoystick.povRight().onTrue(superstructure.setMiddlePos());
        programmerJoystick.povLeft().onTrue(superstructure.setTestHome());
        //operatorJoy.povLeft().onTrue(new ElevatorTest(superstructure));
        //operatorJoy.rightBumper().onTrue(new L4finalPos(superstructure));
    

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        joystick.leftBumper()
            .whileTrue(new IntakeDeploy(coralIntake)
            .andThen(coralIntake.setRollerOpenLoopCommand(-0.7)
            .alongWith(coralGripper.setRollerOpenLoopCommand(0.25))))
        
            .onFalse(coralIntake.setRollerOpenLoopCommand(0)
            .alongWith(coralGripper.setRollerOpenLoopCommand(0.05)));
        
        
        joystick.leftTrigger()
            .whileTrue(coralIntake.setRollerOpenLoopCommand(0.5))

            .onFalse(coralIntake.setRollerOpenLoopCommand(0)
            .alongWith(new IntakeRetract(coralIntake)));            

        
        //joystick.rightBumper().toggleOnTrue()
        //joystick.rightBumper().toggleOnFalse()
        joystick.rightTrigger().whileTrue(coralGripper.setRollerOpenLoopCommand(-0.5)).onFalse(coralGripper.setRollerOpenLoopCommand(0));

        joystick.b().onTrue(superstructure.setTargetL1());
        joystick.a().onTrue(superstructure.setTargetL2());
        joystick.x().onTrue(superstructure.setTargetL3());
        joystick.y().onTrue(superstructure.setTargetL4());

        //Binds the roller intake to the right Bumper
        // programmerJoystick.leftBumper().whileTrue(coralIntake.setRollerOpenLoopCommand(0.5)).onFalse(coralIntake.setRollerOpenLoopCommand(0));
        // programmerJoystick.leftTrigger().whileTrue(coralIntake.setRollerOpenLoopCommand(-0.7)).onFalse(coralIntake.setRollerOpenLoopCommand(0));

        //programmerJoystick.rightBumper().whileTrue(coralGripper.setRollerOpenLoopCommand(0.25)).onFalse(coralGripper.setRollerOpenLoopCommand(0.05));
        
        programmerJoystick.povUp().whileTrue(superstructure.armUp()).onFalse(superstructure.armStop());
        programmerJoystick.povDown().whileTrue(superstructure.armDown()).onFalse(superstructure.armStop());

        // programmerJoystick.a().onTrue(coralIntake.setAngleCommand(-115));
        // programmerJoystick.b().onTrue(coralIntake.setAngleCommand(-1));

        // programmerJoystick.y().onTrue(superstructure.setArmAngle(123));
        // programmerJoystick.x().onTrue(superstructure.setArmAngle(-1));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
