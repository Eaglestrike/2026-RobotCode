package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.concurrent.atomic.AtomicReference;

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
import frc.robot.subsystems.shooter.solver.BallisticConstants;
import frc.robot.subsystems.shooter.solver.BallisticSolver;
import frc.robot.subsystems.shooter.solver.BallisticSolver.SolverMode;
import frc.robot.subsystems.shooter.solver.BallisticConstants.SpeedPreset;
import frc.robot.subsystems.shooter.solver.ShooterSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver.ShotData;

public class Shooter extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;

    // Current target (field-relative 3D pose, set each loop by Superstructure)
    private Pose3d targetPose = null;

    // Ballistic solver parameters (configured once, reused every loop)
    private final BallisticConstants ballisticParams = new BallisticConstants();
    private final ShooterSolver solver;

    // Cached solver result for diagnostics
    private ShotData lastSolverResult = null;

    // Calculated targets (for on-target checks and logging)
    private double cachedFieldTurretDeg = 0;
    private double targetTurretAngleDeg = 0;
    private double targetHoodAngleDeg = ShooterConstants.HoodConstants.DEFAULT_GOAL;
    private double targetFlywheelVel = ShooterConstants.FlywheelConstants.IDLE_VELOCITY;
    private double distanceToTarget = 0;
    private SpeedPreset currentSpeedPreset = SpeedPreset.MEDIUM;
    private double currentLaunchVelocity = BallisticConstants.MEDIUM_SPEED_LAUNCH_VEL;

    // Predicted trajectory for AdvantageScope 3D visualization
    private Pose3d[] predictedTrajectoryPoses = new Pose3d[0];

    // Background solver thread
    private final Thread solverThread;
    private volatile boolean solverRunning = true;
    private final AtomicReference<SolverInput> pendingSolverInput = new AtomicReference<>(null);
    private final AtomicReference<SolverOutput> latestSolverOutput = new AtomicReference<>(null);

    private static final int SOLVER_SUBMIT_PERIOD = 3; // submit every N periodic() calls (~60ms)
    private int periodicCounter = 0;

    private boolean armed = false;
    private volatile boolean directTurret = false;
    private Translation3d cachedShooterPos = new Translation3d();
    private ChassisSpeeds cachedFieldSpeeds = new ChassisSpeeds();

    private record SolverInput(
            Pose3d shooterPose, ChassisSpeeds fieldSpeeds,
            Translation3d target, Pose2d robotPose) {
    }

    private record SolverOutput(
            ShotData shotData, double targetHoodDeg, double fieldTurretDeg,
            double targetRpm) {
    }

    public Shooter(EagleSwerveDrivetrain drive) {
        this(drive, SolverMode.CLIENT);
    }

    public Shooter(EagleSwerveDrivetrain drive, SolverMode solverMode) {
        this.drive = drive;
        turret = new ShooterTurret(drive);
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();
        solver = new BallisticSolver(solverMode);

        solverThread = new Thread(this::solverLoop, "BallisticSolver");
        solverThread.setDaemon(true);
        solverThread.start();

    }

    private static final long RECONNECT_INTERVAL_MS = 3000;

    private void solverLoop() {
        long lastReconnectAttempt = 0;
        while (solverRunning) {
            BallisticSolver bs = (BallisticSolver) solver;
            if (bs.needsReconnect()) {
                long now = System.currentTimeMillis();
                if (now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
                    lastReconnectAttempt = now;
                    System.out.println("[Shooter] Attempting CLIENT reconnect...");
                    bs.tryConnectClient();
                }
            }

            SolverInput input = pendingSolverInput.getAndSet(null);
            if (input == null) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
                continue;
            }
            ShotData result = solver.calculateShot(
                    input.shooterPose, input.fieldSpeeds, input.target);

            double dx = input.target.getX() - input.shooterPose.getX();
            double dy = input.target.getY() - input.shooterPose.getY();
            DogLog.log("Shooter/Solver/InputDist_m", Math.sqrt(dx * dx + dy * dy));
            DogLog.log("Shooter/Solver/ResultRPM", result.rpm());
            DogLog.log("Shooter/Solver/ResultValid", result.validShot());

            double hoodDeg = MathUtil.clamp(result.hoodAngle().getDegrees(),
                    ShooterConstants.HoodConstants.MIN_ANGLE,
                    ShooterConstants.HoodConstants.MAX_ANGLE);
            double fieldTurretDeg;
            if (directTurret) {
                fieldTurretDeg = Math.toDegrees(Math.atan2(dy, dx));
            } else {
                fieldTurretDeg = result.turretAngle().getDegrees();
            }
            latestSolverOutput.set(new SolverOutput(
                    result, hoodDeg, fieldTurretDeg, result.rpm()));
        }
    }

    public SolverMode getSolverMode() {
        return ((BallisticSolver) solver).getMode();
    }

    public void setSolverMode(SolverMode mode) {
        ((BallisticSolver) solver).setMode(mode);
    }

    public boolean isDirectTurret() {
        return directTurret;
    }

    public void setDirectTurret(boolean direct) {
        this.directTurret = direct;
        System.out.println("[Shooter] Direct turret mode: " + direct);
    }

    public Command switchToClientMode() {
        return Commands.runOnce(() -> setSolverMode(SolverMode.CLIENT));
    }

    public Command switchToLocalMode() {
        return Commands.runOnce(() -> setSolverMode(SolverMode.LOCAL));
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
        double cur = turret.getAngle().in(Degrees);
        double error = MathUtil.inputModulus(cur - targetTurretAngleDeg, -180, 180);
        return Math.abs(error) < ShooterConstants.TurretConstants.ANGLE_TOLERANCE.in(Degrees);
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
            pendingSolverInput.set(null);
            lastSolverResult = null;
            predictedTrajectoryPoses = new Pose3d[0];
            periodicCounter = 0;
            logValues();
            return;
        }

        Pose2d robotPose = drive.getState().Pose;

        // Read latest result from background thread (non-blocking, every cycle)
        SolverOutput output = latestSolverOutput.getAndSet(null);
        if (output != null) {
            lastSolverResult = output.shotData();
            targetHoodAngleDeg = output.targetHoodDeg();
            cachedFieldTurretDeg = output.fieldTurretDeg();
            targetFlywheelVel = output.targetRpm();
        }

        // Convert field-relative turret angle to robot-relative using CURRENT heading
        // (every cycle)
        targetTurretAngleDeg = -(cachedFieldTurretDeg - robotPose.getRotation().getDegrees());

        // Throttle: only submit solver input every N cycles (~60ms)
        if (++periodicCounter >= SOLVER_SUBMIT_PERIOD) {
            periodicCounter = 0;

            // Only submit if the previous input was consumed
            if (pendingSolverInput.get() == null) {
                Translation2d offsetField = ShooterConstants.TurretConstants.OFFSET
                        .rotateBy(robotPose.getRotation());
                Translation2d shooterPos2d = robotPose.getTranslation().plus(offsetField);
                Translation3d shooterPos = new Translation3d(
                        shooterPos2d.getX(), shooterPos2d.getY(),
                        BallisticConstants.SHOOTER_HEIGHT);

                Translation3d deltaField = targetPose.getTranslation().minus(shooterPos);
                distanceToTarget = Math.sqrt(deltaField.getX() * deltaField.getX()
                        + deltaField.getY() * deltaField.getY());
                currentSpeedPreset = BallisticConstants.getSpeedPresetForDistance(distanceToTarget);
                currentLaunchVelocity = BallisticConstants.getLaunchVelocityForPreset(currentSpeedPreset);

                Pose3d shooterPose = new Pose3d(shooterPos, new Rotation3d());
                ChassisSpeeds robotSpeeds = drive.getState().Speeds;
                Translation2d fieldVel = new Translation2d(robotSpeeds.vxMetersPerSecond,
                        robotSpeeds.vyMetersPerSecond)
                        .rotateBy(robotPose.getRotation());
                ChassisSpeeds fieldSpeeds = new ChassisSpeeds(fieldVel.getX(), fieldVel.getY(),
                        robotSpeeds.omegaRadiansPerSecond);

                cachedShooterPos = shooterPos;
                cachedFieldSpeeds = fieldSpeeds;

                pendingSolverInput.set(new SolverInput(shooterPose, fieldSpeeds,
                        targetPose.getTranslation(), robotPose));
            }
        }

        // Always command subsystems with cached results (every cycle for smooth motion)
        turret.setAngle(Degrees.of(targetTurretAngleDeg));
        hood.setGoal(Degrees.of(targetHoodAngleDeg));

        boolean solverReady = lastSolverResult != null && lastSolverResult.validShot()
                && targetFlywheelVel > 0;

        if (armed && solverReady) {
            flywheels.setVelocity(targetFlywheelVel);
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
        if (lastSolverResult == null) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        double launchAngleRad = Math.toRadians(lastSolverResult.hoodAngle().getDegrees());
        double turretAngleRad = lastSolverResult.turretAngle().getRadians();

        // Ball's field-frame initial velocity = launch velocity + robot velocity
        double ct = Math.cos(launchAngleRad);
        double vxF = currentLaunchVelocity * ct * Math.cos(turretAngleRad) + fieldSpeeds.vxMetersPerSecond;
        double vyF = currentLaunchVelocity * ct * Math.sin(turretAngleRad) + fieldSpeeds.vyMetersPerSecond;
        double vzF = currentLaunchVelocity * Math.sin(launchAngleRad);

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
        DogLog.log("Shooter/ActualTurretAngle_deg", turret.getAngle().in(Degrees));
        DogLog.log("Shooter/TurretError_deg",
                MathUtil.inputModulus(turret.getAngle().in(Degrees) - targetTurretAngleDeg, -180, 180));
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
        DogLog.log("Shooter/Solver/Mode", getSolverMode().name());
        DogLog.log("Shooter/Solver/ClientConnected",
                getSolverMode() == SolverMode.CLIENT && !((BallisticSolver) solver).needsReconnect());
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
