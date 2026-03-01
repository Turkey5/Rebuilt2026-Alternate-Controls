// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Feed;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDeployerDefaultCommand;
import frc.robot.commands.IntakeExtend;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.ResetManualIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeployer;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
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

    private final CommandXboxController driveTrainController = new CommandXboxController(0);
    private final CommandXboxController shooterController = new CommandXboxController(1);


    public CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Feeder feeder;
    private Rollers rollers;
    private Intake intake;
    private IntakeDeployer intakeDeployer;


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        InitializeSubsystems();
        registerCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("FireShooter",getFireCommand().withTimeout(3.0));

    }

    private Command getFireCommand() {
        return new Feed(feeder).alongWith(new RollerCommand(rollers));
    }

    private Command runToPath(String pathName) {
      //  Command command = AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.271, 3.991, new Rotation2d(Math.toRadians(180.000))), PathConstraints.unlimitedConstraints(12.0));
      //  return command.andThen(getFireCommand().withTimeout(2.0));
      try {
        PathPlannerPath goalPath = PathPlannerPath.fromPathFile(pathName);
        PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindThenFollowPath(goalPath, constraints).andThen(drivetrain.applyRequest(() -> brake)).withTimeout(1);
      } catch(Exception e) {
        return drivetrain.applyRequest(() -> brake);
      }

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveTrainController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveTrainController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveTrainController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driveTrainController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveTrainController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveTrainController.getLeftY(), -driveTrainController.getLeftX()))
        ));

        driveTrainController.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driveTrainController.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveTrainController.back().and(driveTrainController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveTrainController.back().and(driveTrainController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveTrainController.start().and(driveTrainController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveTrainController.start().and(driveTrainController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driveTrainController.y().onTrue(runToPath("Tel Score From Middle"));

        // Reset the field-centric heading on left bumper press.
        driveTrainController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void InitializeSubsystems() {
        try {
            shooter = new Shooter();
            ShooterDefaultCommand shooterCmd = new ShooterDefaultCommand(shooter);
            shooter.setDefaultCommand(shooterCmd);
        } catch(Throwable error) {
            System.out.println(error.getMessage());
        }
        try {
            feeder = new Feeder();
        } catch(Throwable error) {
            System.out.println(error.getMessage());
        }
        try {
            rollers = new Rollers();
        } catch(Throwable error) {
            System.out.println(error.getMessage());
        }
        try {
            intake = new Intake();
        } catch(Throwable error) {
            System.out.println(error.getMessage());
        }
        try {
            intakeDeployer = new IntakeDeployer();  
            intakeDeployer.setDefaultCommand(new IntakeDeployerDefaultCommand(intakeDeployer, shooterController));
            shooterController.rightBumper().onTrue(new IntakeExtend(intakeDeployer));
            shooterController.leftBumper().onTrue(new IntakeRetract(intakeDeployer));
            shooterController.leftStick().onTrue(new ResetManualIntake(intakeDeployer));
        } catch(Throwable error){
            System.out.println(error.getMessage());
        }
        if (rollers != null && feeder != null) {
            shooterController.leftTrigger().whileTrue(new Feed(feeder).alongWith(new RollerCommand(rollers)));
            shooterController.y().onTrue(new Feed(feeder).alongWith(new RollerCommand(rollers)).withTimeout(3));
        } 
        if (intake != null && rollers != null) {
            IntakeCommand intakeCmd = new IntakeCommand(intake);
            RollerCommand rollerCmd = new RollerCommand(rollers);
            shooterController.rightTrigger().whileTrue(intakeCmd.alongWith(rollerCmd));
        }


    }
}
