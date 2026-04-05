package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.RightTrench;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.serialization.Kicker;
import frc.robot.subsystems.serialization.Spindexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.solver.LUTSolver;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class Superstructure extends SubsystemBase {
    public static double turretTrimDegrees = 0;
    public static double hoodTrimDegrees = 0;

    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    private ShootMode currentMode;
    public boolean isShooting;
    private boolean isKickerOverrided;
    private Pose3d targetPose;

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private final IntakeWrist intakeWrist;
    private final Kicker kicker;
    private final Spindexer spindexer;
    private final LED led;
    private final EagleSwerveDrivetrain drivetrain;
    private boolean tuning = false;

    public Optional<Pose2d> getTargetPose2d() {
        if (targetPose == null) {
            return Optional.empty();
        }
        return Optional.of(new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().toRotation2d()));
    }

    public double getExpectedTOF() {
        if (targetPose == null) {
            return 0.0;
        }
        double distance = LUTSolver.getDistanceToTarget(
                new Pose3d(drivetrain.getState().Pose).getTranslation(),
                targetPose.getTranslation()).in(edu.wpi.first.units.Units.Meters);
        return LUTSolver.TOF_MAP.get(distance);
    }

    public Superstructure(Climb climb, Shooter shooter, IntakeRollers intakeRollers, IntakeWrist intakeWrist, Kicker kicker, Spindexer spindexer, EagleSwerveDrivetrain drivetrain, LED led) {
        currentMode = ShootMode.SHOOT;
        isShooting = false;
        this.climb = climb;
        this.shooter = shooter;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.kicker = kicker;
        this.spindexer = spindexer;
        this.led = led;

        DogLog.tunable("Superstructure/Tuning", tuning, value -> tuning = value);
        DogLog.tunable("Superstructure/Shooting", isShooting, value -> setShooting(value));
        DogLog.forceNt.log("Superstructure/TurretTrim", turretTrimDegrees);
        DogLog.forceNt.log("Superstructure/HoodTrim", hoodTrimDegrees);
        this.drivetrain = drivetrain;
        DogLog.forceNt.log("Superstructure/Mode", currentMode.toString());
    }

    public Command initialize() {
        return Commands.sequence(
                duck(),
                led.setColor(Color.kGreen)
        /* climb.moveToStow() */);
    }

    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.parallel(
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.MAX_ANGLE)),
                intakeRollers.start());

    }

    /**
     * @return runOnce command that schedules the intake to stow
     */
    public Command stowIntake() {
        return Commands.sequence(
                // intakeRollers.stop(),
                intakeRollers.eject(),
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE)),
                Commands.runOnce(() -> intakeWrist.setJiggle(false)),
                intakeWrist.retain());
    }

    public Command stopIntakeRollers() {
        return intakeRollers.stop();
    }

    /**
     * Stows everything, including turret, hood, intake, and climber
     */
    public Command stow() {
        return Commands.parallel(
                stowIntake(),
                shooter.stowShooter(),
                Commands.runOnce(() -> setShooting(false))
        /* climb.moveToStow() */);
    }

    public Command ejectIntake() {
        return Commands.sequence(
                intakeWrist.eject(),
                Commands.waitSeconds(0.5),
                intakeRollers.eject());
    }

    public Command jiggleIntake() {
        return Commands.parallel(
                Commands.runOnce(() -> intakeWrist.setJiggle(true)),
                intakeRollers.jiggle())
                .finallyDo(() -> intakeWrist.setJiggle(false));
    }

    public Command duck() {
        return Commands.parallel(
                shooter.duck(),
                Commands.runOnce(() -> {
                    isShooting = false;
                    intakeWrist.setJiggle(false);
                }),
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.MAX_ANGLE)));
    }

    public Command unDuck() {
        return shooter.unDuck();
    }

    public void setMode(ShootMode mode) {
        currentMode = mode;
        DogLog.forceNt.log("Superstructure/Mode", currentMode.toString());
        if (mode == ShootMode.FERRY) {
            led.setColor(Color.kOrange);
        } else if (mode == ShootMode.SHOOT) {
            led.setColor(Color.kGreen);
        }
    }

    public Command changeHoodTrim(DoubleSupplier delta) {
        return runOnce(() -> {
            hoodTrimDegrees += delta.getAsDouble();
            DogLog.forceNt.log("Superstructure/HoodTrim", hoodTrimDegrees);
        });
    }

    public Command changeTurretTrim(DoubleSupplier delta) {
        return runOnce(() -> {
            turretTrimDegrees += delta.getAsDouble();
            DogLog.forceNt.log("Superstructure/TurretTrim", turretTrimDegrees);
        });
    }

    public Command resetShootOffset() {
        return runOnce(() -> {
            turretTrimDegrees = 0;
            hoodTrimDegrees = 0;
            DogLog.forceNt.log("Superstructure/TurretTrim", turretTrimDegrees);
            DogLog.forceNt.log("Superstructure/HoodTrim", hoodTrimDegrees);
        });
    }

    /**
     * Turns shooting on or off
     */
    public void setShooting(boolean shooting) {
        isShooting = shooting;
        shooter.setShooting(shooting);
    }

    // stop kicking but keep shooter/turret running
    // overrided true = no kicking; overrided false = kicking
    public void setKickerOverride(boolean overrided) {
        isKickerOverrided = overrided;
    }

    /**
     * Computes the Pose3d target for ferry mode in front of the alliance wall
     */
    private Pose3d computeFerryTarget() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        boolean isBlue = alliance == DriverStation.Alliance.Blue;
        final double EDGE_OFFSET = 2.0;
        final double X_OFFSET_FROM_WALL = 2.0; // how far from the wall we want to be in ferry mode, in meters
        double targetX = isBlue ? X_OFFSET_FROM_WALL : FieldConstants.fieldLength - X_OFFSET_FROM_WALL;

        // Robot's current Y on the field
        Pose2d robotPose = drivetrain.getState().Pose;
        double robotY = robotPose.getY();

        double targetY = robotY >= FieldConstants.LinesHorizontal.center ? FieldConstants.fieldWidth - EDGE_OFFSET
                : EDGE_OFFSET;

        return new Pose3d(targetX, targetY, RightTrench.height, new Rotation3d());
    }

    public void toggleMode() {
        if (currentMode == ShootMode.FERRY) {
            setMode(ShootMode.SHOOT);
        } else {
            setMode(ShootMode.FERRY);
        }
    }

    public ShootMode getMode() {
        return currentMode;
    }

    public void fastPeriodic() {
        if (currentMode == ShootMode.FERRY) {
            Pose3d ferryTarget = computeFerryTarget();
            shooter.setFacing(ferryTarget, true);
            targetPose = ferryTarget;
            if (tuning) {
                shooter.shootManual();
            }
        } else {
            targetPose = FieldConstants.getAllianceHubPosition();
            shooter.setFacing(targetPose, false);
            if (tuning) {
                shooter.shootManual();
            }
        }

        if (!DriverStation.isEnabled()) {
            setShooting(false);
            kicker.stopMotor();
            spindexer.stopMotor();
            intakeWrist.setJiggle(false);
            return;
        }

        if (isShooting && shooter.isReady() && !isKickerOverrided) { // for ferrying check if trajectory intersects hub
            kicker.startMotor();
            spindexer.startKicking();
        } else {
            kicker.stopMotor();
            spindexer.startMotor(); // this starts at like 0.3 volts don't worry
        }
    }

    @Override
    public void simulationPeriodic() {
        DogLog.log("Sim/ComponentPoses", new Pose3d[] {
                new Pose3d(0, 0, 0, new Rotation3d()),
                shooter.getTurret().getComponentPose3d(),
                shooter.getHood().getComponentPose3d(shooter.getTurret().getComponentPose3d()),
                new Pose3d(0, 0, 0, new Rotation3d()),
                new Pose3d(0, 0, 0, new Rotation3d()),
        });
    }

}