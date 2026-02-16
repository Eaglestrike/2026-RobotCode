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
import frc.robot.subsystems.shooter.ballistics.BallisticConstants;
import frc.robot.subsystems.shooter.ballistics.BallisticConstants.SpeedPreset;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver.SolverResult;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver.Vec3;

public class Shooter extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;

    // Current target (field-relative 3D pose, set each loop by Superstructure)
    private Pose3d targetPose = null;

    // Ballistic solver parameters (configured once, reused every loop)
    private final BallisticConstants ballisticParams = new BallisticConstants();

    // Cached solver result for diagnostics
    private SolverResult lastSolverResult = null;

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
        ballisticParams.dt = 0.005;     // 5 ms RK4 step
        ballisticParams.tMax = 3.0;     // max 3 s flight
        ballisticParams.tolMiss = 0.05; // 5 cm miss tolerance
        ballisticParams.maxIter = 15;
    }

    /**
     * Set the 3D field-relative target to aim at.
     * Called from Superstructure.periodic() every loop.
     * Actual motor commands are issued in this class's periodic().
     */
    public void setFacing(Pose3d target) {
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
        return Math.abs(cur - targetTurretAngleDeg)
                < ShooterConstants.TurretConstants.ANGLE_TOLERANCE.in(Degrees);
    }

    private boolean isHoodOnTarget() {
        double cur = hood.getAngle().in(Degrees);
        return Math.abs(cur - targetHoodAngleDeg)
                < ShooterConstants.HoodConstants.ANGLE_TOLERANCE.in(Degrees);
    }

    private boolean isFlywheelReady() {
        return flywheels.atVelocity();
    }

    private boolean hasSolution() {
        return lastSolverResult != null && lastSolverResult.success;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public ShooterFlywheels getFlywheels() { return flywheels; }
    public ShooterTurret getTurret()       { return turret; }
    public ShooterHood getHood()           { return hood; }


    @Override
    public void periodic() {
        if (targetPose == null) {
            flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            lastSolverResult = null;
            predictedTrajectoryPoses = new Pose3d[0];
            logValues();
            return;
        }

        // ---- 1. Robot state ----
        Pose2d robotPose = drive.getState().Pose;
        var chassisSpeeds = drive.getState().Speeds;
        double robotYawRad = robotPose.getRotation().getRadians();

        // ---- 2. Shooter exit point in field frame ----
        Translation2d offsetField = ShooterConstants.TurretConstants.OFFSET
                .rotateBy(robotPose.getRotation());
        Translation2d shooterPos2d = robotPose.getTranslation().plus(offsetField);
        Translation3d shooterPos = new Translation3d(
                shooterPos2d.getX(), shooterPos2d.getY(),
                BallisticConstants.SHOOTER_HEIGHT);

        // ---- 3. Delta from shooter → target in field frame ----
        Translation3d deltaField = targetPose.getTranslation().minus(shooterPos);

        // ---- 4. Rotate delta into robot-body frame (X = fwd, Y = left) ----
        double cos = Math.cos(-robotYawRad);
        double sin = Math.sin(-robotYawRad);
        double relX = deltaField.getX() * cos - deltaField.getY() * sin;
        double relY = deltaField.getX() * sin + deltaField.getY() * cos;
        double relZ = deltaField.getZ();

        // ---- 5. Relative velocity of target w.r.t. shooter ----
        //  v_shooter_field = v_chassis + omega × r_offset
        double vxF = chassisSpeeds.vxMetersPerSecond;
        double vyF = chassisSpeeds.vyMetersPerSecond;
        double omega = chassisSpeeds.omegaRadiansPerSecond;

        // Tangential velocity from yaw rate: ω × r  (2D cross product)
        double vTangX = -omega * offsetField.getY();
        double vTangY =  omega * offsetField.getX();

        double shooterVxF = vxF + vTangX;
        double shooterVyF = vyF + vTangY;

        // Into body frame, negated (target moves opposite to shooter)
        double relVelX = -(shooterVxF * cos - shooterVyF * sin);
        double relVelY = -(shooterVxF * sin + shooterVyF * cos);

        // ---- 6. Horizontal distance → speed preset ----
        distanceToTarget = Math.sqrt(relX * relX + relY * relY);
        currentSpeedPreset = BallisticConstants.getSpeedPresetForDistance(distanceToTarget);
        currentLaunchVelocity = BallisticConstants.getLaunchVelocityForPreset(currentSpeedPreset);
        targetFlywheelVel = BallisticConstants.getRPMForPreset(currentSpeedPreset);

        // ---- 7. Solve ballistics ----
        Vec3 relPos = new Vec3(relX, relY, relZ);
        Vec3 relVel = new Vec3(relVelX, relVelY, 0);

        lastSolverResult = BallisticSolver.solve(
                relPos, relVel,
                currentLaunchVelocity,
                BallisticConstants.DEFAULT_DRAG_COEFFICIENT,
                ballisticParams);

        if (lastSolverResult.success || Double.isFinite(lastSolverResult.theta)) {
            double launchAngleDeg = Math.toDegrees(lastSolverResult.theta);
            targetTurretAngleDeg = Math.toDegrees(lastSolverResult.phi);
            targetHoodAngleDeg = BallisticConstants.launchAngleToHoodAngle(launchAngleDeg);
            targetHoodAngleDeg = MathUtil.clamp(targetHoodAngleDeg,
                    ShooterConstants.HoodConstants.MIN_ANGLE,
                    ShooterConstants.HoodConstants.MAX_ANGLE);
            targetTurretAngleDeg = MathUtil.inputModulus(targetTurretAngleDeg, -180, 180);
        } else {
            // Fallback: point directly at target with crude gravity compensation
            targetTurretAngleDeg = Math.toDegrees(Math.atan2(relY, relX));
            double fallbackLaunch = Math.toDegrees(Math.atan2(relZ, distanceToTarget))
                    + distanceToTarget * 1.5;
            targetHoodAngleDeg = BallisticConstants.launchAngleToHoodAngle(
                    MathUtil.clamp(fallbackLaunch,
                            ShooterConstants.HoodConstants.MIN_ANGLE,
                            ShooterConstants.HoodConstants.MAX_ANGLE));
        }

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
        if (lastSolverResult == null || !Double.isFinite(lastSolverResult.theta)
                || !Double.isFinite(lastSolverResult.tStar) || lastSolverResult.tStar <= 0) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        double theta = lastSolverResult.theta;
        double phi   = lastSolverResult.phi;
        double tFlight = lastSolverResult.tStar;

        // Launch velocity in body frame
        double ct = Math.cos(theta);
        double vxB = currentLaunchVelocity * ct * Math.cos(phi);
        double vyB = currentLaunchVelocity * ct * Math.sin(phi);
        double vzB = currentLaunchVelocity * Math.sin(theta);

        // Body → field rotation
        double cosY = Math.cos(robotYawRad);
        double sinY = Math.sin(robotYawRad);
        double vxF = vxB * cosY - vyB * sinY;
        double vyF = vxB * sinY + vyB * cosY;
        double vzF = vzB;

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
        DogLog.log("Shooter/Solver/Success", hasSolution());
        if (lastSolverResult != null) {
            DogLog.log("Shooter/Solver/Miss_m", lastSolverResult.miss);
            DogLog.log("Shooter/Solver/TimeOfFlight_s", lastSolverResult.tStar);
            DogLog.log("Shooter/Solver/Iterations", lastSolverResult.iterations);
            DogLog.log("Shooter/Solver/LaunchAngle_deg",
                    Math.toDegrees(lastSolverResult.theta));
            DogLog.log("Shooter/Solver/YawAngle_deg",
                    Math.toDegrees(lastSolverResult.phi));
        }

        // 3D visualization poses
        if (targetPose != null) {
            DogLog.log("Shooter/TargetPose3d", new Pose3d[] { targetPose });
        }
        if (predictedTrajectoryPoses.length > 0) {
            DogLog.log("Shooter/PredictedTrajectory", predictedTrajectoryPoses);
        }
    }
}
