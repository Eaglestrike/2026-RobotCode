package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.RightTrench;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.serialization.Feeder;
import frc.robot.subsystems.serialization.Hopper;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Superstructure extends SubsystemBase {
    public static Translation2d shootOffset = new Translation2d(0, 0.2);

    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    /**
     * Height, in meters, above the trench to use as the ferry target Z (near the
     * ball's apex).
     */
    private static final double FERRY_HEIGHT_OFFSET = 0.10;

    private ShootMode currentMode;
    public boolean isShooting;

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private final IntakeWrist intakeWrist;
    private final Feeder feeder;
    private final Hopper hopper;
    private final EagleSwerveDrivetrain drivetrain;
    private boolean tuning = false;

    public Superstructure(Climb climb, Shooter shooter, IntakeRollers intakeRollers,
            IntakeWrist intakeWrist, Feeder feeder, Hopper hopper, EagleSwerveDrivetrain drivetrain) {
        currentMode = ShootMode.SHOOT;
        isShooting = false;
        this.climb = climb;
        this.shooter = shooter;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.feeder = feeder;
        this.hopper = hopper;

        DogLog.tunable("Superstructure/ShootOffsetX", shootOffset.getX(),
                value -> shootOffset = new Translation2d(value, shootOffset.getY()));
        DogLog.tunable("Superstructure/ShootOffsetY", shootOffset.getY(),
                value -> shootOffset = new Translation2d(shootOffset.getX(), value));

        DogLog.tunable("Superstructure/Tuning", tuning, value -> tuning = value);
        DogLog.tunable("Superstructure/Shooting", isShooting, value -> setShooting(value));

        this.drivetrain = drivetrain;
    }

    public Command initialize() {
        return runOnce(() -> {
        });
    }

    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.sequence(
                intakeRollers.startHopperOut(),
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.MAX_ANGLE)),
                Commands.waitUntil(intakeWrist::clearedHopper),
                intakeRollers.start());

    }

    /**
     * @return runOnce command that schedules the intake to stow
     */
    public Command stowIntake() {
        return Commands.sequence(
                intakeRollers.stop(),
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE)));
    }

    public Command stopIntakeRollers() {
        return intakeRollers.stop();
    }

    /**
     * Stows everything, including turret, hood, intake, and climber
     */
    public Command stow() {
        return Commands.parallel(
                intakeRollers.stop(),
                intakeWrist.requestPosition(Degrees.of(
                        IntakeConstants.Wrist.STOW_ANGLE)),
                shooter.stowShooter(),
                runOnce(() -> isShooting = false),
                climb.moveTo(ClimbConstants.ClimbState.STOWED));
    }

    public Command ejectIntake() {
        return Commands.sequence(
                intakeWrist.eject(),
                Commands.waitSeconds(0.5),
                intakeRollers.eject());
    }

    public Command jiggleIntake() {
        return Commands.parallel(
                intakeWrist.jiggle(),
                intakeRollers.jiggle());
    }

    /**
     * Command to climb to a specified level
     * 
     * @param level The level to climb to
     * @return SequenceCommand that performs the climbing action
     */
    public Command climb(ClimbConstants.ClimbState level) {
        return Commands.sequence(
                stow(),
                climb.moveTo(level));
    }

    public ClimbConstants.ClimbState getClimbState() {
        return climb.getState();
    }

    public void setMode(ShootMode mode) {
        currentMode = mode;
    }

    /**
     * Turns shooting on or off
     */
    public void setShooting(boolean shooting) {
        isShooting = shooting;
        shooter.setShooting(shooting);
    }

    private Pose3d applyShootOffset(Pose3d target) {
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue;
        double xSign = isBlue ? 1.0 : -1.0;
        double ySign = isBlue ? 1.0 : -1.0;
        return new Pose3d(
                target.getX() + shootOffset.getX() * xSign,
                target.getY() + shootOffset.getY() * ySign,
                target.getZ(),
                target.getRotation());
    }

    /**
     * Computes the Pose3d target for ferry mode (look at the README for sketchy
     * explanation of the logic).
     */
    private Pose3d computeFerryTarget() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        boolean isBlue = alliance == DriverStation.Alliance.Blue;

        // X: the line over the bump/trench on our alliance's side of the field
        double targetX = isBlue
                ? FieldConstants.LinesVertical.hubCenter
                : FieldConstants.LinesVertical.oppHubCenter;

        // Robot's current Y on the field
        Pose2d robotPose = drivetrain.getState().Pose;
        double robotY = robotPose.getY();

        // Clamp Y into the open trench zone on the correct side of the hub.
        // If the robot is above the field center-line (left side), aim into the left
        // trench opening;
        // otherwise aim into the right trench opening.
        double targetY;
        if (robotY >= FieldConstants.LinesHorizontal.center) {
            // Left trench opening: from leftTrenchOpenEnd (just above the bump) to
            // leftTrenchOpenStart (field wall)
            double bumpMiddle = (FieldConstants.LinesHorizontal.leftBumpStart
                    + FieldConstants.LinesHorizontal.leftBumpEnd) / 2;
            targetY = MathUtil.clamp(robotY,
                    bumpMiddle,
                    FieldConstants.LinesHorizontal.leftTrenchOpenStart);
        } else {
            // Right trench opening: from rightTrenchOpenEnd (field wall, 0) to
            // rightTrenchOpenStart (just below bump)
            double bumpMiddle = (FieldConstants.LinesHorizontal.rightBumpStart
                    + FieldConstants.LinesHorizontal.rightBumpEnd) / 2;
            targetY = MathUtil.clamp(robotY,
                    FieldConstants.LinesHorizontal.rightTrenchOpenEnd,
                    bumpMiddle);
        }

        return new Pose3d(targetX, targetY, RightTrench.height + FERRY_HEIGHT_OFFSET, new Rotation3d());
    }

    @Override
    public void periodic() {
        if (tuning && isShooting) {
            shooter.shootManual();
            feeder.startMotor();
            hopper.startKicking();
            return;
        } else if(!isShooting) {
            shooter.getFlywheels().setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            feeder.stopMotor();
            hopper.startMotor();
            return;
        }

        if (currentMode == ShootMode.FERRY) {
            Pose3d ferryTarget = computeFerryTarget();
            DogLog.log("Superstructure/FerryTarget", ferryTarget);
            shooter.setFacing(ferryTarget);
        } else {
            shooter.setFacing(applyShootOffset(FieldConstants.getAllianceHubPosition()));
        }

        if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
            isShooting = false;
            feeder.stopMotor();
            hopper.stopMotor();
            return;
        }

        if (isShooting && shooter.isReady()) {
            feeder.startMotor();
            hopper.startKicking();
        } else {
            feeder.stopMotor();
            hopper.startMotor(); // this starts at like 0.3 volts don't worry
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