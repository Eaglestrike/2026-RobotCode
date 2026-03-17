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

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private final IntakeWrist intakeWrist;
    private final Kicker kicker;
    private final Spindexer spindexer;
    private final LED led;
    private final EagleSwerveDrivetrain drivetrain;
    private boolean tuning = false;

    public Superstructure(Climb climb, Shooter shooter, IntakeRollers intakeRollers,
            IntakeWrist intakeWrist, Kicker kicker, Spindexer spindexer, EagleSwerveDrivetrain drivetrain, LED led) {
        currentMode = ShootMode.SHOOT;
        isShooting = false;
        this.climb = climb;
        this.shooter = shooter;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.kicker = kicker;
        this.spindexer = spindexer;
        this.led = led;

        DogLog.tunable("Superstructure/TurretTrimDegrees", turretTrimDegrees,
                value -> turretTrimDegrees = value);
        DogLog.tunable("Superstructure/HoodTrimDegrees", hoodTrimDegrees,
                value -> hoodTrimDegrees = value);

        DogLog.tunable("Superstructure/Tuning", tuning, value -> tuning = value);
        DogLog.tunable("Superstructure/Shooting", isShooting, value -> setShooting(value));
        this.drivetrain = drivetrain;
    }

    public Command initialize() {
        return Commands.sequence(
                led.setColor(Color.kGreen)
        /* climb.moveToStow() */);
    }

    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.sequence(
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.MAX_ANGLE)),
                intakeRollers.start());

    }

    /**
     * @return runOnce command that schedules the intake to stow
     */
    public Command stowIntake() {
        return Commands.sequence(
                intakeRollers.stop(),
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
                Commands.runOnce(() -> isShooting = false)
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
        return Commands.sequence(
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
        if (mode == ShootMode.FERRY) {
            led.setColor(Color.kOrange);
        } else if (mode == ShootMode.SHOOT) {
            led.setColor(Color.kGreen);
        }
    }

    public Command changeHoodTrim(DoubleSupplier delta) {
        return runOnce(() -> hoodTrimDegrees += delta.getAsDouble());
    }

    public Command changeTurretTrim(DoubleSupplier delta) {
        return runOnce(() -> turretTrimDegrees += delta.getAsDouble());
    }

    public Command resetShootOffset() {
        return runOnce(() -> {
            turretTrimDegrees = 0;
            hoodTrimDegrees = 0;
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

    // getters for trims so other classes can read current trim values
    public double getTurretTrimDegrees() {
        return turretTrimDegrees;
    }

    public double getHoodTrimDegrees() {
        return hoodTrimDegrees;
    }

    /**
     * Computes the Pose3d target for ferry mode (look at the README for sketchy
     * explanation of the logic).
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

    private void autoSetShooting() {
        // TODO: do hub / alliance based logic to turn shooting on or off
        // TODO: make this trigger based so it only calls once
    }

    public void toggleMode() {
        if (currentMode == ShootMode.FERRY) {
            setMode(ShootMode.SHOOT);
        } else {
            setMode(ShootMode.FERRY);
        }
    }

    public void fastPeriodic() {
        autoSetShooting();
        DogLog.forceNt.log("Superstructure/Mode", currentMode.toString());
        if (currentMode == ShootMode.FERRY) {
            Pose3d ferryTarget = computeFerryTarget();
            DogLog.log("Superstructure/FerryTarget", ferryTarget);
            shooter.setFacing(ferryTarget);
        } else {
            shooter.setFacing(FieldConstants.getAllianceHubPosition());
            if (tuning) {
                shooter.shootManual();
            }
        }

        if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
            isShooting = false;
            kicker.stopMotor();
            spindexer.stopMotor();
            intakeWrist.setJiggle(false);
            return;
        }

        if (isShooting && shooter.isReady() && !isKickerOverrided) { // for ferrying check if trajectory intersects hub
            kicker.startMotor();
            spindexer.startKicking();
            // intakeWrist.setJiggle(true);
        } else {
            kicker.stopMotor();
            spindexer.startMotor(); // this starts at like 0.3 volts don't worry
            // intakeWrist.setJiggle(false);
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