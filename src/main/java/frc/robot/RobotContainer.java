// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import frc.robot.FieldConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.shooter.ShooterFlywheels;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.solver.BallisticSolver.SolverMode;
import frc.robot.subsystems.vision.GamePieceDetector;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.AutoCollectCommand;

import com.pathplanner.lib.path.PathConstraints;

public class RobotContainer {
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

        // solver mode chooser
        private final SendableChooser<SolverMode> solverModeChooser = new SendableChooser<>();

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
                        feeder, hopper);
        private final Vision vision = new Vision(drivetrain);

        // TODO: Update camera name and class ID to match PhotonVision GUI
        private static final String GAME_PIECE_CAMERA_NAME = "FrontCamera";
        private static final int FUEL_CLASS_ID = 0;

        private final GamePieceDetector gamePieceDetector = new GamePieceDetector(
                        GAME_PIECE_CAMERA_NAME, FUEL_CLASS_ID);

        // AutoCollect target — web dashboard writes, robot reads
        private final NetworkTable collectTable = NetworkTableInstance.getDefault().getTable("AutoCollect");
        private final DoubleSubscriber collectTargetX = collectTable.getDoubleTopic("TargetX")
                        .subscribe(8.27);
        private final DoubleSubscriber collectTargetY = collectTable.getDoubleTopic("TargetY")
                        .subscribe(4.0);
        private final DoublePublisher robotPoseXPub = collectTable.getDoubleTopic("RobotX").publish();
        private final DoublePublisher robotPoseYPub = collectTable.getDoubleTopic("RobotY").publish();
        private final DoublePublisher robotHeadingPub = collectTable.getDoubleTopic("RobotHeading").publish();
        private final DoubleArrayPublisher ballsXPub = collectTable.getDoubleArrayTopic("BallsX").publish();
        private final DoubleArrayPublisher ballsYPub = collectTable.getDoubleArrayTopic("BallsY").publish();
        private final BooleanSubscriber goSub = collectTable.getBooleanTopic("Go").subscribe(false);
        private final BooleanPublisher goPub = collectTable.getBooleanTopic("Go").publish();

        public RobotContainer() {
                gamePieceDetector.setRobotPoseSupplier(() -> drivetrain.getState().Pose);

                drivetrain.resetPose(new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180)));
                configureBindings();

                NamedCommands.registerCommand("ClimbL1", superstructure.climb(ClimbConstants.ClimbState.L1));
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
                NamedCommands.registerCommand("AlignLeftTower",
                                drivetrain.alignPID(() -> FieldConstants.getAllianceTowerPosition(true)));
                NamedCommands.registerCommand("AlignRightTower",
                                drivetrain.alignPID(() -> FieldConstants.getAllianceTowerPosition(false)));
                NamedCommands.registerCommand("AutoCollect",
                                autoCollect(new Pose2d(8.27, 4.0, Rotation2d.fromDegrees(0))));

                // Dashboard "Go" button → start AutoCollect
                new Trigger(() -> goSub.get())
                                .onTrue(Commands.runOnce(() -> {
                                        goPub.set(false);
                                        CommandScheduler.getInstance().schedule(autoCollectFromDashboard());
                                }));

                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Chooser", autoChooser);

                if (!RobotBase.isSimulation()) {
                        solverModeChooser.setDefaultOption("CLIENT (server)", SolverMode.CLIENT);
                        solverModeChooser.addOption("LOCAL (roboRIO)", SolverMode.LOCAL);
                        SmartDashboard.putData("Solver Mode", solverModeChooser);
                        new Trigger(() -> solverModeChooser.getSelected() != shooter.getSolverMode())
                                        .onTrue(Commands.runOnce(() -> shooter.setSolverMode(
                                                        solverModeChooser.getSelected()))
                                                        .ignoringDisable(true));
                }
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
                        drivetrain.resetPose(
                                        new Pose2d(FieldConstants.fieldLength - 0.5, FieldConstants.fieldWidth - 0.5,
                                                        Rotation2d.fromDegrees(180)));
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

                operatorController.button(3).whileTrue(intakeRollers.setManualVoltage(operatorController::getRightY))
                                .onFalse(intakeRollers.setManualVoltage(() -> 0));
                operatorController.button(3).whileTrue(intakeWrist.setManualVoltage(operatorController::getLeftY))
                                .onFalse(intakeWrist.setManualVoltage(() -> 0));

                operatorController.a()
                                .whileTrue(climb.setManualVoltage(operatorController::getLeftY,
                                                operatorController::getRightY))
                                .onFalse(climb.setManualVoltage(() -> 0, () -> 0));

                // rightJoystick.button(3)
                // .onTrue(drivetrain.alignPID(() ->
                // FieldConstants.getAllianceTowerPosition(true)));
                rightJoystick.button(4)
					.onTrue(climbL1(() -> false))
					.onFalse(drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
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
                        return Commands.sequence( // TODO: Redo superstructure.climb
							// superstructure.climb(ClimbConstants.ClimbState.L1),
							new PrintCommand("Aligning to " + left.getAsBoolean() + " tower"),
							drivetrain.alignPID(() -> FieldConstants.getAllianceTowerPosition(left.getAsBoolean())),
							Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityX(0).withVelocityY(1))),
							Commands.waitSeconds(0.5),
							Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0)))
							// superstructure.climb(ClimbConstants.ClimbState.STOWED)
                        );	
                }

        public Trigger deadbandTrigger(DoubleSupplier axis, double deadband) {
                return new Trigger(() -> Math.abs(axis.getAsDouble()) > deadband);
        }

        /**
         * Publishes robot pose to NT for the web dashboard.
         * Call this from robotPeriodic or a default command.
         */
        public void publishDashboardData() {
                gamePieceDetector.periodic();

                Pose2d pose = drivetrain.getState().Pose;
                robotPoseXPub.set(pose.getX());
                robotPoseYPub.set(pose.getY());
                robotHeadingPub.set(pose.getRotation().getDegrees());

                List<Translation2d> balls = gamePieceDetector.getDetectedPositions();
                double[] bx = new double[balls.size()];
                double[] by = new double[balls.size()];
                for (int i = 0; i < balls.size(); i++) {
                        bx[i] = balls.get(i).getX();
                        by[i] = balls.get(i).getY();
                }
                ballsXPub.set(bx);
                ballsYPub.set(by);
        }

        /**
         * Creates an AutoCollect command using the target point set from the
         * web dashboard (via NetworkTables).
         */
        public Command autoCollectFromDashboard() {
                Pose2d target = new Pose2d(
                                collectTargetX.get(),
                                collectTargetY.get(),
                                new Rotation2d());
                return new AutoCollectCommand(drivetrain, gamePieceDetector, superstructure, target);
        }

        public Command autoCollect(Pose2d targetPose) {
                return new AutoCollectCommand(drivetrain, gamePieceDetector, superstructure, targetPose);
        }

        public Command autoCollect(Pose2d targetPose, PathConstraints constraints) {
                return new AutoCollectCommand(drivetrain, gamePieceDetector, superstructure, targetPose, constraints);
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
