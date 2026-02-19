package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.solver.BallisticConstants;
import frc.robot.subsystems.shooter.solver.BallisticSolver;
import frc.robot.subsystems.shooter.solver.BallisticConstants.SpeedPreset;
import frc.robot.subsystems.shooter.solver.ShooterSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver.ShotData;

public class Shooter extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;

    // Current target (field-relative 3D pose, set each loop by Superstructure)
    private Translation3d targetPose = null;

    // Ballistic solver parameters (configured once, reused every loop)
    private final BallisticConstants ballisticParams = new BallisticConstants();
    private final ShooterSolver solver;

    // Cached solver result for diagnostics
    private ShotData lastSolverResult = null;

    // Calculated targets (for on-target checks and logging)
    private double targetTurretAngleDeg = 0;
    private double targetHoodAngleDeg = 0;
    private double targetFlywheelVel = ShooterConstants.FlywheelConstants.IDLE_VELOCITY;
    private double distanceToTarget = 0;
    private SpeedPreset currentSpeedPreset = SpeedPreset.MEDIUM;
    private double currentLaunchVelocity = BallisticConstants.MEDIUM_SPEED_LAUNCH_VEL;

    // Predicted trajectory for AdvantageScope 3D visualization
    private Pose3d[] predictedTrajectoryPoses = new Pose3d[0];

    public Shooter(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        turret = new ShooterTurret(drive);
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();

        // Solver configuration
        ballisticParams.highArc = true; // low-arc for speed
        ballisticParams.g = 9.80665;
        ballisticParams.dt = 0.005; // 5 ms RK4 step
        ballisticParams.tMax = 3.0; // max 3 s flight
        ballisticParams.tolMiss = 0.05; // 5 cm miss tolerance
        ballisticParams.maxIter = 15;

        solver = new BallisticSolver(ballisticParams, BallisticConstants.DEFAULT_DRAG_COEFFICIENT);
    }

    /**
     * Set the 3D field-relative target to aim at.
     * Called from Superstructure.periodic() every loop.
     * Actual motor commands are issued in this class's periodic().
     */
    public void setFacing(Translation3d target) {
        this.targetPose = target;
    }

    /** Stow turret/hood/flywheel (command for auto / teleop transitions). */
    public Command stowShooter() {
        return Commands.parallel(
                turret.requestPosition(Rotations.of(ShooterConstants.TurretConstants.STOW_POSITION)),
                hood.goToPos(Radians.of(ShooterConstants.HoodConstants.STOW_ANGLE)),
                runOnce(() -> flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY)));
    }

    /** True when turret + hood + flywheel are all at their targets. */
    public boolean isReady() {
        return isTurretOnTarget() && isHoodOnTarget() && isFlywheelReady();
    }

    public boolean isTurretOnTarget() {
        double cur = turret.getAngle().in(Degrees);
        return Math.abs(cur - targetTurretAngleDeg) < ShooterConstants.TurretConstants.ANGLE_TOLERANCE.in(Degrees);
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
        double robotYawRad = robotPose.getRotation().getRadians();

        Translation2d offsetField = ShooterConstants.TurretConstants.OFFSET
                .rotateBy(robotPose.getRotation());
        Translation2d shooterPos2d = robotPose.getTranslation().plus(offsetField);
        Translation3d shooterPos = new Translation3d(
                shooterPos2d.getX(), shooterPos2d.getY(),
                BallisticConstants.SHOOTER_HEIGHT);

        // ---- Solve ballistics via ShooterSolver interface ----
        lastSolverResult = solver.calculateShot(
                shooterPos, drive.getState().Speeds,
                targetPose);

        // ---- Extract results from ShotData ----
        targetHoodAngleDeg = MathUtil.clamp(lastSolverResult.hoodAngle().getDegrees(),
                ShooterConstants.HoodConstants.MIN_ANGLE,
                ShooterConstants.HoodConstants.MAX_ANGLE);
        targetTurretAngleDeg = MathUtil.inputModulus(lastSolverResult.turretAngle().getDegrees(), -180, 180);
        targetFlywheelVel = lastSolverResult.rpm();

        // Update distance for diagnostics
        Translation3d deltaField = targetPose.minus(shooterPos);
        distanceToTarget = Math.sqrt(deltaField.getX() * deltaField.getX()
                + deltaField.getY() * deltaField.getY());
        currentSpeedPreset = BallisticConstants.getSpeedPresetForDistance(distanceToTarget);
        currentLaunchVelocity = BallisticConstants.getLaunchVelocityForPreset(currentSpeedPreset);

        // ---- 8. Command subsystems directly (no Command scheduling in periodic) ----
        turret.setAngle(Degrees.of(targetTurretAngleDeg));
        hood.setGoal(Degrees.of(targetHoodAngleDeg));
        flywheels.setVelocity(targetFlywheelVel);

        // ---- 9. Build predicted trajectory for 3D visualization ----
        buildTrajectoryVisualization(shooterPos, robotYawRad);

        // ---- 10. Log ----
        logValues();
    }

    // ================================================================
    // Trajectory visualization (for AdvantageScope 3D Field)
    // ================================================================
    private void buildTrajectoryVisualization(Translation3d shooterPos, double robotYawRad) {
        if (lastSolverResult == null || !lastSolverResult.validShot()) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        // Recover launch angles from ShotData
        double launchAngleRad = Math.toRadians(
                BallisticConstants.hoodAngleToLaunchAngle(lastSolverResult.hoodAngle().getDegrees()));
        double turretAngleRad = lastSolverResult.turretAngle().getRadians();

        // Estimate time of flight from distance / launch velocity
        Translation3d predictedTarget = lastSolverResult.predictedTarget();
        double dx = predictedTarget.getX() - shooterPos.getX();
        double dy = predictedTarget.getY() - shooterPos.getY();
        double horizDist = Math.sqrt(dx * dx + dy * dy);
        double tFlight = horizDist / Math.max(currentLaunchVelocity * Math.cos(launchAngleRad), 1e-6);
        if (!Double.isFinite(tFlight) || tFlight <= 0) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        // Launch velocity in field frame (turret angle from solver is already
        // field-relative)
        double ct = Math.cos(launchAngleRad);
        double vxF = currentLaunchVelocity * ct * Math.cos(turretAngleRad);
        double vyF = currentLaunchVelocity * ct * Math.sin(turretAngleRad);
        double vzF = currentLaunchVelocity * Math.sin(launchAngleRad);

        // Sample ~20 points along the trajectory
        int n = 20;
        double dtVis = tFlight / n;
        predictedTrajectoryPoses = new Pose3d[n + 1];

        double px = shooterPos.getX(), py = shooterPos.getY(), pz = shooterPos.getZ();
        double vx = vxF, vy = vyF, vz = vzF;
        double kDrag = BallisticConstants.DEFAULT_DRAG_COEFFICIENT;
        double g = ballisticParams.g;

        for (int i = 0; i <= n; i++) {
            predictedTrajectoryPoses[i] = new Pose3d(px, py, Math.max(pz, 0), new Rotation3d());
            if (i < n) {
                double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
                double df = kDrag * speed;
                vx -= df * vx * dtVis;
                vy -= df * vy * dtVis;
                vz -= (g + df * vz) * dtVis;
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
        DogLog.log("Shooter/HasTarget", targetPose != null);
        DogLog.log("Shooter/DistanceToTarget", distanceToTarget);
        DogLog.log("Shooter/SpeedPreset", currentSpeedPreset.name());
        DogLog.log("Shooter/LaunchVelocity_mps", currentLaunchVelocity);
        DogLog.log("Shooter/TargetTurretAngle_deg", targetTurretAngleDeg);
        DogLog.log("Shooter/TargetHoodAngle_deg", targetHoodAngleDeg);
        DogLog.log("Shooter/TargetFlywheelVel_rps", targetFlywheelVel);
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

        // 3D visualization poses
        if (targetPose != null) {
            DogLog.log("Shooter/TargetPose3d",
                    new Pose3d[] { new Pose3d(targetPose, new Rotation3d()) });
        }
        if (predictedTrajectoryPoses.length > 0) {
            DogLog.log("Shooter/PredictedTrajectory", predictedTrajectoryPoses);
        }
    }
}
