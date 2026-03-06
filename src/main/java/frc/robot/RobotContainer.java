// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.drive.EagleSwerveTelemetry;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.serialization.Feeder;
import frc.robot.subsystems.serialization.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFlywheels;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private static final boolean ENABLE_CLIMB_COMMAND = false;
    /** Meters from the tower to align to (for center of the robot) */
    private static final double CLIMB_TOWER_DISTANCE = 0.5;
    /** Meters per second when moving towards the pole */
    private static final double CLIMB_SPEED = 0.5;

    public static final double AXIS_DEADBAND = 0.02;
    public static final CANBus MECHANISM_CANBUS = new CANBus("Mechanisms");
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
    private final CommandXboxController operatorController = new CommandXboxController(2);
    private final CommandXboxController buttonBoard = new CommandXboxController(3);

    // auto
    private final SendableChooser<Command> autoChooser;

    // Button Board constants
    private final int CLIMB_BUTTON = 4;
    private final int INTAKE_BUTTON = 5;
    private final int HOOD_BUTTON = 2;
    private final int TURRET_BUTTON = 1;
    private final int SERIALIZER_BUTTON = 3;

    // subsystems
    private final Shooter shooter = new Shooter(drivetrain);
    private final ShooterTurret turret = shooter.getTurret();
    private final ShooterFlywheels shooterFlywheels = shooter.getFlywheels();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final IntakeWrist intakeWrist = new IntakeWrist();
    private final Feeder feeder = new Feeder();
    private final Hopper hopper = new Hopper();
    private final Climb climb = new Climb();
    private final Superstructure superstructure = new Superstructure(climb, shooter, intakeRollers, intakeWrist,
            feeder, hopper, drivetrain);
    private final Vision vision = new Vision(drivetrain);

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("ClimbL1Left", climbL1(() -> true));
        NamedCommands.registerCommand("ClimbL1Right", climbL1(() -> false));
        NamedCommands.registerCommand("SetModeFerry",
                Commands.runOnce(() -> superstructure.setMode(ShootMode.FERRY)));
        NamedCommands.registerCommand("SetModeShoot",
                Commands.runOnce(() -> superstructure.setMode(ShootMode.SHOOT)));
        NamedCommands.registerCommand("StartShooting",
                Commands.runOnce(() -> superstructure.setShooting(true)));
        NamedCommands.registerCommand("StopShooting",
                Commands.runOnce(() -> superstructure.setShooting(false)));
        NamedCommands.registerCommand("Intake", superstructure.intake());
        NamedCommands.registerCommand("StowIntake", superstructure.stowIntake());
        NamedCommands.registerCommand("Stow", superstructure.stow());

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    var driveMult = slowModeOn ? DriveConstants.SLOW_MODE_MULT : 1;
                    return drive
                            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT
                                    * driveMult)
                            .withRotationalDeadband(MaxAngularRate
                                    * DriveConstants.DRIVE_DEADBAND_MULT
                                    * driveMult)
                            .withVelocityX(-leftJoystick.getY() * MaxSpeed * driveMult)
                            .withVelocityY(-leftJoystick.getX() * MaxSpeed * driveMult)
                            .withRotationalRate(-rightJoystick.getX() * MaxAngularRate
                                    * driveMult);
                }));

        rightJoystick.button(2)
                .onTrue(Commands.runOnce(() -> slowModeOn = true))
                .onFalse(Commands.runOnce(() -> slowModeOn = false));

        operatorController.button(8).onTrue(drivetrain.runOnce(() -> {
            System.out.println("Zeroing drive");
            drivetrain.seedFieldCentric();
            drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0, 0)));
        }).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.button(7).and(buttonBoard.button(CLIMB_BUTTON))
                .onTrue(climb.zero().ignoringDisable(true));

        operatorController.button(7).and(buttonBoard.button(INTAKE_BUTTON))
                .onTrue(intakeWrist.zero().ignoringDisable(true));

        operatorController.button(7).and(buttonBoard.button(HOOD_BUTTON))
                .onTrue(shooter.getHood().zero().ignoringDisable(true));

        operatorController.button(7).and(buttonBoard.button(TURRET_BUTTON))
                .onTrue(shooter.getTurret().zero().ignoringDisable(true));

        // -------- FOR TESTING ONLY --------
        operatorController.button(7).and(operatorController.x())
                .onTrue(intakeWrist.zero().ignoringDisable(true));

        operatorController.button(7).and(operatorController.a())
                .onTrue(shooter.getHood().zero().ignoringDisable(true));

        operatorController.button(7).and(operatorController.b())
                .onTrue(shooter.getTurret().zero().ignoringDisable(true));

        operatorController.button(3).whileTrue(intakeRollers.setManualVoltage(operatorController::getRightY))
                .onFalse(intakeRollers.setManualVoltage(() -> 0));
        operatorController.button(3).whileTrue(intakeWrist.setManualVoltage(operatorController::getLeftY))
                .onFalse(intakeWrist.setManualVoltage(() -> 0));

        operatorController.a()
                .whileTrue(climb.setManualVoltage(operatorController::getLeftY,
                        operatorController::getRightY))
                .onFalse(climb.setManualVoltage(() -> 0, () -> 0));

        rightJoystick.button(4)
                .onTrue(climbL1(() -> true))
                .onFalse(drivetrain.applyRequest(
                        () -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
        // ---------------------------------

        rightJoystick.trigger().onTrue(superstructure.intake());
        leftJoystick.trigger().onTrue(superstructure.stopIntakeRollers());
        // eject intake
        leftJoystick.button(3).onTrue(superstructure.ejectIntake());
        // stow all
        leftJoystick.button(2).onTrue(superstructure.stow());
        // jiggle intake
        rightJoystick.button(3).onTrue(superstructure.jiggleIntake());

        // Manual Commands
        buttonBoard.button(INTAKE_BUTTON).and(deadbandTrigger(operatorController::getRightY, AXIS_DEADBAND))
                .whileTrue(intakeRollers.setManualVoltage(operatorController::getRightY))
                .onFalse(intakeRollers.setManualVoltage(() -> 0));

        buttonBoard.button(INTAKE_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(intakeWrist.setManualVoltage(operatorController::getLeftY))
                .onFalse(intakeWrist.setManualVoltage(() -> 0));

        buttonBoard.button(CLIMB_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(climb.setManualVoltage(operatorController::getLeftY,
                        operatorController::getRightY))
                .onFalse(climb.setManualVoltage(() -> 0, () -> 0));

        buttonBoard.button(TURRET_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(turret.setManualVoltage(operatorController::getLeftY))
                .onFalse(turret.setManualVoltage(() -> 0));

        buttonBoard.button(TURRET_BUTTON).and(deadbandTrigger(operatorController::getRightY, AXIS_DEADBAND))
                .whileTrue(shooterFlywheels.setManualVoltage(operatorController::getRightY))
                .onFalse(shooterFlywheels.setManualVoltage(() -> 0));

        buttonBoard.button(HOOD_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(shooter.getHood().setManualVoltage(operatorController::getLeftY))
                .onFalse(shooter.getHood().setManualVoltage(() -> 0));

        buttonBoard.button(SERIALIZER_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(hopper.setManualVoltage(operatorController::getLeftY))
                .onFalse(hopper.setManualVoltage(() -> 0));

        operatorController.leftBumper()
                .onTrue(Commands.runOnce(() -> superstructure.setShooting(!superstructure.isShooting)));
        // Should the kicker and flywheels be on the same button, like hood getRightY or
        // serializer getRightY? Since they need to spin simultaneously to launch
    }

    public Command getOffTower() {
        if (superstructure.getClimbState() == null
                || superstructure.getClimbState() == ClimbConstants.ClimbState.STOWED) {
            return Commands.print("Not currently climbing, cannot get off tower");
        }

        int direction = drivetrain.getState().Pose.getY() > FieldConstants.LinesHorizontal.center ? 1 : -1;
        return Commands.sequence(
                superstructure.climb(ClimbConstants.ClimbState.STOWED),
                Commands.runOnce(() -> drivetrain.setControl(
                        drive.withVelocityY(MaxSpeed * direction
                                * DriveConstants.GET_OFF_TOWER_SPEED_MULT))),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityY(0.0))));
    }

    public Command climbL1(BooleanSupplier left) {
        if (!ENABLE_CLIMB_COMMAND) {
            return Commands.print("Climb command disabled");
        }

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        Pose3d towerPolePose = FieldConstants.getAllianceTowerPosition(left.getAsBoolean());
        // DogLog.log("Climb/TowerPolePose", towerPolePose);
        double multiplier = left.getAsBoolean() ^ (alliance == DriverStation.Alliance.Red) ? 1 : -1;
        DogLog.log("Climb/DirectionMultiplier", multiplier);
        Pose3d alignPose = new Pose3d(
                towerPolePose.getTranslation().plus(new Translation3d(0, multiplier * CLIMB_TOWER_DISTANCE, 0)),
                towerPolePose.getRotation());
        DogLog.log("Climb/AlignPose", alignPose);

        return Commands.sequence( // TODO: Redo superstructure.climb
                // superstructure.climb(ClimbConstants.ClimbState.L1),
                new PrintCommand("Aligning to " + left.getAsBoolean() + " tower"),
                drivetrain.alignPID(() -> alignPose.toPose2d()),
                Commands.runOnce(
                        () -> drivetrain.setControl(drive.withVelocityX(0).withVelocityY(CLIMB_SPEED * multiplier))),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0)))
        // superstructure.climb(ClimbConstants.ClimbState.STOWED)
        );
    }

    public Trigger deadbandTrigger(DoubleSupplier axis, double deadband) {
        return new Trigger(() -> Math.abs(axis.getAsDouble()) > deadband);
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
                superstructure.initialize(),
                autoChooser.getSelected());
    }

    public Command getTeleopCommand() {
        return superstructure.initialize();
    }

    public EagleSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
