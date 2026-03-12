// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.lib.zone.RebuiltZones;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveMode;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.drive.EagleSwerveTelemetry;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.serialization.Kicker;
import frc.robot.subsystems.serialization.Spindexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFlywheels;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    /**
     * Meters from the tower to align to (for center of the robot)
     */
    private static final double CLIMB_TOWER_DISTANCE = 0.5;
    /**
     * Meters per second when moving towards the pole
     */
    private static final double CLIMB_SPEED = 1;
    /**
     * Trim the target pose
     */
    private static final double TRIM_DISTANCE = 0.1;

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

    private final PIDController alignYController = new PIDController(8, 0, 0.05);
    private final SwerveRequest.FieldCentricFacingAngle trenchDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5, 0, 0)
            .withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final EagleSwerveTelemetry logger = new EagleSwerveTelemetry(MaxSpeed);
    private final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private DriveMode driveMode = DriveMode.NORMAL;

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

    private final int POS_1_BUTTON = 6;
    private final int POS_2_BUTTON = 7;
    private final int POS_3_BUTTON = 8;
    private final int POS_4_BUTTON = 9;

    private final int TOGGLE_MODE_BUTTON = 10;
    private final int SHOOT_BUTTON = 11;

    // subsystems
    private final Shooter shooter = new Shooter(drivetrain);
    private final ShooterTurret turret = shooter.getTurret();
    private final ShooterFlywheels shooterFlywheels = shooter.getFlywheels();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final IntakeWrist intakeWrist = new IntakeWrist();
    private final Kicker kicker = new Kicker();
    private final Spindexer spindexer = new Spindexer();
    private final Climb climb = new Climb();
    private final LED led = new LED();
    private final Superstructure superstructure = new Superstructure(climb, shooter, intakeRollers, intakeWrist,
            kicker, spindexer, drivetrain, led);
    private final Vision vision = new Vision(drivetrain);

    public RobotContainer() {
        configureBindings();
        configureZoneBindings();

        NamedCommands.registerCommand("ClimbL1Left",
                Commands.defer(() -> climbL1(), java.util.Set.of()));
        NamedCommands.registerCommand("ClimbL1Right",
                Commands.defer(() -> climbL1(), java.util.Set.of()));
        NamedCommands.registerCommand("SetModeFerry",
                Commands.defer(() -> Commands.runOnce(() -> superstructure.setMode(ShootMode.FERRY)),
                        java.util.Set.of()));
        NamedCommands.registerCommand("SetModeShoot",
                Commands.defer(() -> Commands.runOnce(() -> superstructure.setMode(ShootMode.SHOOT)),
                        java.util.Set.of()));
        NamedCommands.registerCommand("StartShooting",
                Commands.defer(() -> Commands.runOnce(() -> superstructure.setShooting(true)), java.util.Set.of()));
        NamedCommands.registerCommand("StopShooting",
                Commands.defer(() -> Commands.runOnce(() -> superstructure.setShooting(false)), java.util.Set.of()));
        NamedCommands.registerCommand("Intake",
                Commands.defer(() -> superstructure.intake(), java.util.Set.of()));
        NamedCommands.registerCommand("StowIntake",
                Commands.defer(() -> superstructure.stowIntake(), java.util.Set.of()));
        NamedCommands.registerCommand("Stow",
                Commands.defer(() -> superstructure.stow(), java.util.Set.of()));
        NamedCommands.registerCommand("Duck",
                Commands.defer(() -> superstructure.duck(), java.util.Set.of()));
        NamedCommands.registerCommand("Unduck",
                Commands.defer(() -> superstructure.unDuck(), java.util.Set.of()));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    if (driveMode == DriveMode.SLOW || driveMode == DriveMode.NORMAL) {
                        var driveMult = driveMode == DriveMode.SLOW ? DriveConstants.SLOW_MODE_MULT : 1;
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
                    }
                    if (driveMode == DriveMode.TRENCH_LOCK) {
                        Pose2d pose = drivetrain.getState().Pose;
                        double veloY = alignYController.calculate(pose.getY(), drivetrain.getTrenchY());
                        // return trenchDrive
                        // .withVelocityX(-leftJoystick.getY() * MaxSpeed)
                        // .withVelocityY(veloY);
                        drive
                                .withVelocityX(-leftJoystick.getY() * MaxSpeed)
                                .withVelocityY(
                                        veloY)
                                .withRotationalRate(-rightJoystick.getX() * MaxAngularRate);
                        // .withTargetDirection(drivetrain.getTrenchLockAngle())
                    }
                    return drive
                            .withVelocityX(-leftJoystick.getY() * MaxSpeed)
                            .withVelocityY(-leftJoystick.getX() * MaxSpeed)
                            .withRotationalRate(-rightJoystick.getX() * MaxAngularRate);
                }));

        rightJoystick.button(2)
                .onTrue(Commands.runOnce(() -> driveMode = DriveMode.SLOW))
                .onFalse(Commands.runOnce(() -> driveMode = DriveMode.NORMAL));

        operatorController.button(8).onTrue(drivetrain.runOnce(() -> {
            System.out.println("Zeroing drive");
            drivetrain.seedFieldCentric();
            drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0, 0)));
        }).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

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

        // ---------------------------------

        rightJoystick.trigger().onTrue(superstructure.intake());
        leftJoystick.trigger().onTrue(superstructure.stopIntakeRollers());
        // eject intake
        leftJoystick.button(3).onTrue(superstructure.ejectIntake());
        // stow all
        leftJoystick.button(2).onTrue(superstructure.stow());
        // jiggle intake
        rightJoystick.button(3).onTrue(superstructure.jiggleIntake());

        // -------- Manual Commands --------
        buttonBoard.button(INTAKE_BUTTON).and(deadbandTrigger(operatorController::getRightY, AXIS_DEADBAND))
                .whileTrue(intakeRollers.setManualVoltage(operatorController::getRightY))
                .onFalse(intakeRollers.setManualVoltage(() -> 0));

        buttonBoard.button(INTAKE_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(intakeWrist.setManualVoltage(operatorController::getLeftY))
                .onFalse(intakeWrist.setManualVoltage(() -> 0));

        buttonBoard.button(CLIMB_BUTTON).and(deadbandTrigger(operatorController::getLeftY, AXIS_DEADBAND))
                .whileTrue(climb.setManualVoltage(operatorController::getLeftY))
                .onFalse(climb.setManualVoltage(() -> 0));

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
                .whileTrue(spindexer.setManualVoltage(operatorController::getLeftY))
                .onFalse(spindexer.setManualVoltage(() -> 0));
        // ---------------------------------

        // ------ Stow Manual Commands ------
        buttonBoard.button(INTAKE_BUTTON).and(operatorController.rightTrigger())
                .onTrue(superstructure.stowIntake());

        buttonBoard.button(HOOD_BUTTON).and(operatorController.rightTrigger())
                .onTrue(shooter.stowShooter());

        buttonBoard.button(TURRET_BUTTON).and(operatorController.rightTrigger())
                .onTrue(shooter.stowShooter());

        // ---------------------------------

        buttonBoard.button(SHOOT_BUTTON)
                .onTrue(Commands.runOnce(() -> superstructure.setShooting(!superstructure.isShooting))).debounce(0.1);

        buttonBoard.button(TOGGLE_MODE_BUTTON)
                .onTrue(Commands.runOnce(() -> superstructure.toggleMode())).debounce(0.1);

        operatorController.rightBumper()
                .onTrue(Commands.runOnce(() -> superstructure.toggleManualOverride())).debounce(0.1);

        operatorController.leftBumper()
                .toggleOnTrue(superstructure.duck())
                .toggleOnFalse(superstructure.unDuck());

        operatorController.povUp()
                .onTrue(superstructure.changeOffsetY(() -> TRIM_DISTANCE)).debounce(0.1);
        operatorController.povDown()
                .onTrue(superstructure.changeOffsetY(() -> -TRIM_DISTANCE)).debounce(0.1);
        operatorController.povRight()
                .onTrue(superstructure.changeOffsetX(() -> TRIM_DISTANCE)).debounce(0.1);
        operatorController.povLeft()
                .onTrue(superstructure.changeOffsetX(() -> -TRIM_DISTANCE)).debounce(0.1);
        // Should the kicker and flywheels be on the same button, like hood getRightY or
        // serializer getRightY? Since they need to spin simultaneously to launch
    }

    private void configureZoneBindings() {
        Trigger inTowerZoneTrigger = RebuiltZones.TOWER_ZONES
                .contains(() -> drivetrain.getState().Pose)
                .debounce(0.1);
        inTowerZoneTrigger
                .onTrue(Commands.runOnce(() -> {
                    DogLog.log("Zones/InTowerZone", true);
                    superstructure.setKickerOverride(true);
                }))
                .onFalse(Commands.runOnce(() -> {
                    DogLog.log("Zones/InTowerZone", false);
                    superstructure.setKickerOverride(false);
                }));
        Trigger inTrenchZoneTrigger = RebuiltZones.TRENCH_ZONES
                .willContain(() -> drivetrain.getState().Pose, () -> drivetrain.getState().Speeds, Seconds.of(0.5));
        inTrenchZoneTrigger
                .onTrue(Commands.sequence(
                        Commands.runOnce(() -> {
                            DogLog.log("Zones/InTrenchZone", true);
                            driveMode = DriveMode.TRENCH_LOCK;
                        }),
                        superstructure.duck()))
                .onFalse(Commands.sequence(
                        Commands.runOnce(() -> {
                            DogLog.log("Zones/InTrenchZone", false);
                            driveMode = DriveMode.NORMAL;
                        }),
                        superstructure.unDuck()));
    }

    public Command climbL1() {
        return Commands.defer(() -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

            boolean absoluteLeft = drivetrain.getState().Pose.getY() > FieldConstants.LinesHorizontal.center;
            boolean isLeft = alliance == DriverStation.Alliance.Blue ? absoluteLeft : !absoluteLeft;

            Pose3d towerPolePose = FieldConstants.getAllianceTowerPosition(isLeft);
            DogLog.log("Climb/TowerPolePose", towerPolePose);
            double multiplier = isLeft != (alliance == DriverStation.Alliance.Red) ? 1 : -1;
            DogLog.log("Climb/DirectionMultiplier", multiplier);
            Pose3d alignPose = new Pose3d(
                    towerPolePose.getTranslation().plus(new Translation3d(0, multiplier * CLIMB_TOWER_DISTANCE, 0)),
                    towerPolePose.getRotation());
            DogLog.log("Climb/AlignPose", alignPose);

            return Commands.sequence(
                    new PrintCommand("Aligning to " + (isLeft ? "left" : "right") + " tower"),
                    drivetrain.alignPID(() -> alignPose.toPose2d()),
                    new PrintCommand("Moving in..."),
                    Commands.runOnce(
                            () -> drivetrain
                                    .setControl(drive.withVelocityX(0).withVelocityY(CLIMB_SPEED * multiplier))),
                    Commands.waitSeconds(1.0),
                    climb.moveToUp(),
                    new PrintCommand("Finished climbing"),
                    Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0)))
            );
        }, Set.of());
    }

    public Trigger deadbandTrigger(DoubleSupplier axis, double deadband) {
        return new Trigger(() -> Math.abs(axis.getAsDouble()) > deadband);
    }

    public Command getAutonomousCommand() {
        return Commands.parallel(
                superstructure.initialize(),
                autoChooser.getSelected().asProxy());
    }

    public Command getTeleopCommand() {
        return superstructure.initialize();
    }

    public EagleSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
