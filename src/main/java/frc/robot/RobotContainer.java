// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.drive.EagleSwerveTelemetry;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.serialization.Feeder;
import frc.robot.subsystems.serialization.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;


public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // swerve
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT)
            .withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
    private final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private boolean slowModeOn = false;

    // controllers
    private final CommandJoystick leftJoystick = new CommandJoystick(0);
    private final CommandJoystick rightJoystick = new CommandJoystick(1);
    // private final CommandXboxController operatorController = new
    // CommandXboxController(2);
    // private final CommandXboxController buttonBoard = new
    // CommandXboxController(3);

    // auto
    private final SendableChooser<Command> autoChooser;

    // subsystems
    private final Shooter shooter = new Shooter(drivetrain);
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final IntakeWrist intakeWrist = new IntakeWrist();
    private final Feeder feeder = new Feeder();
    private final Hopper hopper = new Hopper();
    private final Climb climb = new Climb();
    private final Superstructure superstructure = new Superstructure(climb, shooter, intakeRollers, intakeWrist, feeder, hopper);

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("ClimbL1", superstructure.climb(ClimbConstants.ClimbState.L1));
        NamedCommands.registerCommand("SetModeFerry", Commands.runOnce(() -> superstructure.setMode(ShootMode.FERRY)));
        NamedCommands.registerCommand("SetModeShoot", Commands.runOnce(() -> superstructure.setMode(ShootMode.SHOOT)));
        NamedCommands.registerCommand("StartShooting", Commands.runOnce(() -> superstructure.setShooting(true)));
        NamedCommands.registerCommand("StopShooting", Commands.runOnce(() -> superstructure.setShooting(false)));
        NamedCommands.registerCommand("Intake", superstructure.intake());
        NamedCommands.registerCommand("StowIntake", superstructure.stowIntake());
        NamedCommands.registerCommand("Stow", superstructure.stow());
        NamedCommands.registerCommand("AlignLeftTower", drivetrain.alignPID(() -> FieldConstants.getAllianceTowerPosition(true)));
        NamedCommands.registerCommand("AlignRightTower", drivetrain.alignPID(() -> FieldConstants.getAllianceTowerPosition(false)));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    var driveMult = slowModeOn ? DriveConstants.SLOW_MODE_MULT : 1;
                    return drive
                            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT * driveMult)
                            .withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT * driveMult)
                            .withVelocityX(-leftJoystick.getY() * MaxSpeed * driveMult)
                            .withVelocityY(-leftJoystick.getX() * MaxSpeed * driveMult)
                            .withRotationalRate(-rightJoystick.getX() * MaxAngularRate * driveMult);
                }));

        rightJoystick.button(2)
                .onTrue(Commands.runOnce(() -> slowModeOn = true))
                .onFalse(Commands.runOnce(() -> slowModeOn = false));

        rightJoystick.button(3).onTrue(drivetrain.runOnce(() -> {
            System.out.println("Zeroing drive");
            drivetrain.seedFieldCentric();
            drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0, 0)));
        }).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getOffTower() {
        if (superstructure.getClimbState() == null) {
            return Commands.print("Not currently climbing, cannot get off tower");
        }

        int direction = drivetrain.getState().Pose.getY() > 4 ? 1 : -1; // Replace 4 with 1/2 field width
        return Commands.sequence(
                superstructure.climb(ClimbConstants.ClimbState.STOWED),
                Commands.runOnce(() -> drivetrain.setControl(
                        drive.withVelocityY(MaxSpeed * direction * DriveConstants.GET_OFF_TOWER_SPEED_MULT))),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityY(0.0))));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public EagleSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
