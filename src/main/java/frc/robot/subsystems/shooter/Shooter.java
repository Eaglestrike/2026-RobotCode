package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.desmos.DesmosSolver;
import frc.robot.subsystems.shooter.solver.BallisticConstants;
import frc.robot.subsystems.shooter.solver.ShooterSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver.ShotData;

public class Shooter extends SubsystemBase {
    public static final boolean DEBUG = true;

    public static double velocityMultiplier = 0.9; // To scale flywheel velocity

    private final EagleSwerveDrivetrain drive;
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;

    // Current target (field-relative 3D pose, set each loop by Superstructure)
    private Pose3d targetPose = null;

    private final ShooterSolver solver = new DesmosSolver();

    // Cached solver result for diagnostics
    private ShotData lastSolverResult = null;

    // Calculated targets (for on-target checks and logging)
    private double cachedFieldTurretDeg = 0;
    private double targetTurretAngleDeg = 0;
    private double targetHoodAngleDeg = ShooterConstants.HoodConstants.DEFAULT_GOAL;
    private double targetFlywheelVel = ShooterConstants.FlywheelConstants.IDLE_VELOCITY;
    private double distanceToTarget = 0;

    // Predicted trajectory for AdvantageScope 3D visualization
    private Pose3d[] predictedTrajectoryPoses = new Pose3d[0];

    private static final int LOG_EVERY_N_LOOPS = 10;
    private boolean debug = false;
    private int logCounter = 0;

    private boolean armed = false;
    private boolean isShooting = false;
    private Translation3d cachedShooterPos = new Translation3d();
    private ChassisSpeeds cachedFieldSpeeds = new ChassisSpeeds();

    public Shooter(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        turret = new ShooterTurret(drive);
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();

        DogLog.tunable("Shooter/VelocityMultiplier", velocityMultiplier, x -> velocityMultiplier = x);
    }

    /**
     * Set the 3D field-relative target to aim at.
     * Called from Superstructure.periodic() every loop.
     * Actual motor commands are issued in this class's periodic().
     */
    public void setFacing(Pose3d targetPose) {
        this.targetPose = targetPose;
    }

    public void setArmed(boolean armed) {
        this.armed = armed;
    }

    public void setShooting(boolean shooting) {
        this.isShooting = shooting;
    }

    /** Stow turret/hood/flywheel (command for auto / teleop transitions). */
    public Command stowShooter() {
        return Commands.parallel(
                turret.requestPosition(Rotations.of(ShooterConstants.TurretConstants.STOW_POSITION)),
                hood.goToPos(Degrees.of(ShooterConstants.HoodConstants.STOW_ANGLE)),
                runOnce(() -> flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY)));
    }

    public boolean isReady() {
        return isTurretOnTarget() && isHoodOnTarget() && isFlywheelReady();
    }

    public boolean isTurretOnTarget() {
        return turret.atTarget();
    }

    private boolean isHoodOnTarget() {
        double cur = hood.getAngle().in(Degrees);
        return Math.abs(cur - targetHoodAngleDeg) < ShooterConstants.HoodConstants.ANGLE_TOLERANCE.in(Degrees);
    }

    private boolean isFlywheelReady() {
        return flywheels.atVelocity();
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public ShooterFlywheels getFlywheels() {
        return flywheels;
    }

    public ShooterTurret getTurret() {
        return turret;
    }

    public ShooterHood getHood() {
        return hood;
    }

    @Override
    public void periodic() {
        if (targetPose == null) {
            flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            lastSolverResult = null;
            predictedTrajectoryPoses = new Pose3d[0];
            logValues();
            return;
        }

        Pose2d robotPose = drive.getState().Pose;

        // Compute shooter position in field frame
        Translation2d offsetField = ShooterConstants.TurretConstants.OFFSET
                .rotateBy(robotPose.getRotation());
        Translation2d shooterPos2d = robotPose.getTranslation().plus(offsetField);
        Translation3d shooterPos = new Translation3d(
                shooterPos2d.getX(), shooterPos2d.getY(),
                BallisticConstants.SHOOTER_HEIGHT);

        Translation3d deltaField = targetPose.getTranslation().minus(shooterPos);
        distanceToTarget = Math.sqrt(deltaField.getX() * deltaField.getX()
                + deltaField.getY() * deltaField.getY());

        // Compute field-relative robot velocity
        ChassisSpeeds robotSpeeds = drive.getState().Speeds;
        Translation2d fieldVel = new Translation2d(robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond)
                .rotateBy(robotPose.getRotation());
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(fieldVel.getX(), fieldVel.getY(),
                robotSpeeds.omegaRadiansPerSecond);

        cachedShooterPos = shooterPos;
        cachedFieldSpeeds = fieldSpeeds;

        Pose3d shooterPose = new Pose3d(shooterPos, new Rotation3d());
        ShotData result = solver.calculateShot(shooterPose, fieldSpeeds,
                targetPose.getTranslation());

        lastSolverResult = result;
        if (result.validShot()) {
            targetHoodAngleDeg = MathUtil.clamp(result.hoodAngle().getDegrees(),
                    ShooterConstants.HoodConstants.MIN_ANGLE,
                    ShooterConstants.HoodConstants.MAX_ANGLE);
            cachedFieldTurretDeg = result.turretAngle().getDegrees();
            targetFlywheelVel = result.rpm();
        }

        // Convert field-relative turret angle to robot-relative using current heading
        targetTurretAngleDeg = -(cachedFieldTurretDeg - robotPose.getRotation().getDegrees());

        // Command subsystems
        turret.setAngle(Degrees.of(targetTurretAngleDeg));
        hood.setGoal(Degrees.of(targetHoodAngleDeg));

        if (isShooting) {
            flywheels.setVelocity(targetFlywheelVel * velocityMultiplier);
        } else {
            flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
        }

        buildTrajectoryVisualization(cachedShooterPos, cachedFieldSpeeds);
        logValues();
    }

    // ================================================================
    // Trajectory visualization (for AdvantageScope 3D Field)
    // ================================================================
    private void buildTrajectoryVisualization(Translation3d shooterPos, ChassisSpeeds fieldSpeeds) {
        if (lastSolverResult == null || !lastSolverResult.validShot()) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        double launchAngleRad = Math.toRadians(lastSolverResult.hoodAngle().getDegrees());
        double turretAngleRad = lastSolverResult.turretAngle().getRadians();

        // Recover launch velocity from RPM → RPS → ball vel (m/s)
        double launchVelocity = ShooterConstants.flywheelRPSToBallVel(lastSolverResult.rpm() / 60.0);

        // Ball's field-frame initial velocity = launch velocity + robot velocity
        double ct = Math.cos(launchAngleRad);
        double vxF = launchVelocity * ct * Math.cos(turretAngleRad) + fieldSpeeds.vxMetersPerSecond;
        double vyF = launchVelocity * ct * Math.sin(turretAngleRad) + fieldSpeeds.vyMetersPerSecond;
        double vzF = launchVelocity * Math.sin(launchAngleRad);

        // Estimate time of flight from distance / horizontal speed
        Translation3d predictedTarget = lastSolverResult.predictedTarget();
        double dx = predictedTarget.getX() - shooterPos.getX();
        double dy = predictedTarget.getY() - shooterPos.getY();
        double horizDist = Math.sqrt(dx * dx + dy * dy);
        double horizSpeed = Math.sqrt(vxF * vxF + vyF * vyF);
        double tFlight = horizDist / Math.max(horizSpeed, 1e-6);
        if (!Double.isFinite(tFlight) || tFlight <= 0) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        // Sample ~20 points along the trajectory (gravity-only, consistent with DesmosSolver)
        int n = 20;
        double dtVis = tFlight / n;
        predictedTrajectoryPoses = new Pose3d[n + 1];

        double px = shooterPos.getX(), py = shooterPos.getY(), pz = shooterPos.getZ();
        double vx = vxF, vy = vyF, vz = vzF;
        double g = 9.81;

        for (int i = 0; i <= n; i++) {
            predictedTrajectoryPoses[i] = new Pose3d(px, py, Math.max(pz, 0), new Rotation3d());
            if (i < n) {
                // Gravity-only integration (no drag — matches DesmosSolver model)
                vz -= g * dtVis;
                px += vx * dtVis;
                py += vy * dtVis;
                pz += vz * dtVis;
            }
        }
    }

    // ================================================================
    // Logging
    // ================================================================
    private void logValues() {
        if (!DEBUG || logCounter++ < LOG_EVERY_N_LOOPS) {
            return;
        }

        logCounter = 0;

        DogLog.log("Shooter/HasTarget", targetPose != null);
        DogLog.log("Shooter/DistanceToTarget", distanceToTarget);
        DogLog.log("Shooter/TargetTurretAngle_deg", targetTurretAngleDeg);
        DogLog.log("Shooter/FieldTurretAngle_deg", cachedFieldTurretDeg);
        DogLog.log("Shooter/RobotYaw_deg", drive.getState().Pose.getRotation().getDegrees());
        DogLog.log("Shooter/TargetHoodAngle_deg", targetHoodAngleDeg);
        DogLog.log("Shooter/ActualHoodAngle_deg", hood.getAngle().in(Degrees));
        DogLog.log("Shooter/TargetFlywheelVel_rpm", targetFlywheelVel);
        DogLog.log("Shooter/TurretOnTarget", isTurretOnTarget());
        DogLog.log("Shooter/HoodOnTarget", isHoodOnTarget());
        DogLog.log("Shooter/FlywheelReady", isFlywheelReady());
        DogLog.log("Shooter/ReadyToShoot", isReady());

        // Solver diagnostics
        DogLog.log("Shooter/Solver/Success", lastSolverResult != null && lastSolverResult.validShot());
        if (lastSolverResult != null) {
            DogLog.log("Shooter/Solver/HoodAngle_deg", lastSolverResult.hoodAngle().getDegrees());
            DogLog.log("Shooter/Solver/TurretAngle_deg", lastSolverResult.turretAngle().getDegrees());
            DogLog.log("Shooter/Solver/RPM", lastSolverResult.rpm());
        }

        // 3D visualization poses (field-absolute, with odom)
        Pose2d robotPose = drive.getState().Pose;
        Pose3d robotPose3d = new Pose3d(
                robotPose.getX(), robotPose.getY(), 0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        Pose3d turretLocal = turret.getComponentPose3d();
        Pose3d hoodLocal = hood.getComponentPose3d(turretLocal);
        DogLog.log("Shooter/TurretPose3d", new Pose3d[] { robotPose3d.transformBy(toTransform(turretLocal)) });
        DogLog.log("Shooter/HoodPose3d", new Pose3d[] { robotPose3d.transformBy(toTransform(hoodLocal)) });

        if (targetPose != null) {
            DogLog.log("Shooter/TargetPose3d",
                    new Pose3d[] { targetPose });
        }
        if (predictedTrajectoryPoses.length > 0) {
            DogLog.log("Shooter/PredictedTrajectory", predictedTrajectoryPoses);
        }
    }

    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }
}
