package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ballistics.BallisticConstants;
import frc.robot.subsystems.shooter.ballistics.BallisticConstants.SpeedPreset;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver.SolverResult;
import frc.robot.subsystems.shooter.ballistics.BallisticSolver.Vec3;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

/**
 * ShooterController - Unified aiming controller for Turret + Hood + Flywheel
 * 
 * Uses BallisticSolver for accurate trajectory calculation with:
 * - Gravity
 * - Air drag (quadratic)
 * - Moving target lead
 */
public class ShooterController {
    private final EagleSwerveDrivetrain drive;
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;

    // Current target
    private Pose3d targetPose = null;
    private boolean enabled = false;

    // Tunable target for testing (via NetworkTables)
    private double tunableTargetX = 5.0;
    private double tunableTargetY = 0.0;
    private double tunableTargetZ = 2.0;
    private boolean useTunableTarget = false;

    // Calculated values
    private double targetTurretAngleDeg = 0;
    private double targetHoodAngleDeg = 0;
    private double targetFlywheelRPM = FlywheelConstants.IDLE_VELOCITY;
    private double distanceToTarget = 0;
    private SpeedPreset currentSpeedPreset = SpeedPreset.MEDIUM;
    private double currentLaunchVelocity = BallisticConstants.MEDIUM_SPEED_LAUNCH_VEL;

    // Ballistic solver result cache
    private SolverResult lastSolverResult = null;

    // Tolerances
    private static final double TURRET_TOLERANCE_DEG = 2.0;
    private static final double HOOD_TOLERANCE_DEG = 1.0;

    // Ballistic solver parameters
    private final BallisticConstants ballisticParams = new BallisticConstants();

    public ShooterController(EagleSwerveDrivetrain drive, ShooterTurret turret,
            ShooterHood hood, ShooterFlywheels flywheels) {
        this.drive = drive;
        this.turret = turret;
        this.hood = hood;
        this.flywheels = flywheels;

        // Configure ballistic solver
        ballisticParams.highArc = true; // use high arc (parabolic) trajectory
        ballisticParams.g = 9.80665;
        ballisticParams.dt = 0.005; // 5ms integration step
        ballisticParams.tMax = 3.0; // max 3 seconds flight time
        ballisticParams.tolMiss = 0.05; // 5cm tolerance
        ballisticParams.maxIter = 15;

        // Setup tunable target inputs for testing via NetworkTables
        DogLog.tunable("ShooterController/TunableTarget/X", tunableTargetX, x -> tunableTargetX = x);
        DogLog.tunable("ShooterController/TunableTarget/Y", tunableTargetY, y -> tunableTargetY = y);
        DogLog.tunable("ShooterController/TunableTarget/Z", tunableTargetZ, z -> tunableTargetZ = z);
        DogLog.tunable("ShooterController/TunableTarget/Enable", false, enable -> {
            useTunableTarget = enable;
            if (enable) {
                setTarget(new Pose3d(tunableTargetX, tunableTargetY, tunableTargetZ, new Rotation3d()));
                setEnabled(true);
            }
        });
    }

    /**
     * Set the target to aim at
     * 
     * @param target The 3D pose of the target (e.g., Hub)
     */
    public void setTarget(Pose3d target) {
        this.targetPose = target;
    }

    /**
     * Enable or disable the aiming controller
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Check if the controller is enabled (for debugging)
     */
    public boolean isEnabled() {
        return this.enabled;
    }

    /**
     * Call this periodically to update all aiming calculations and commands
     */
    public void update() {
        // Update target from tunable inputs if enabled
        if (useTunableTarget) {
            targetPose = new Pose3d(tunableTargetX, tunableTargetY, tunableTargetZ, new Rotation3d());
            enabled = true;
        }

        // When disabled or no drivetrain, don't touch subsystems (allow manual testing)
        if (!enabled || drive == null) {
            lastSolverResult = null;
            return;
        }

        if (targetPose == null) {
            // Idle mode - maintain idle flywheel speed only when controller is enabled
            flywheels.setVelocity(FlywheelConstants.IDLE_VELOCITY);
            lastSolverResult = null;
            return;
        }

        // Get robot pose and velocity
        Pose2d robotPose = drive.getState().Pose;
        var chassisSpeeds = drive.getState().Speeds;

        // Calculate shooter position in field coordinates
        Translation2d shooterPosition2d = robotPose.getTranslation()
                .plus(TurretConstants.OFFSET.rotateBy(robotPose.getRotation()));
        Translation3d shooterPosition = new Translation3d(
                shooterPosition2d.getX(),
                shooterPosition2d.getY(),
                BallisticConstants.SHOOTER_HEIGHT);

        // Calculate target position relative to shooter (in shooter frame)
        Translation3d targetPosition = targetPose.getTranslation();
        Translation3d relativePosition = targetPosition.minus(shooterPosition);

        // Convert to shooter-relative coordinates (rotate by robot heading)
        double robotYaw = robotPose.getRotation().getRadians();
        double cosYaw = Math.cos(-robotYaw);
        double sinYaw = Math.sin(-robotYaw);

        double relX = relativePosition.getX() * cosYaw - relativePosition.getY() * sinYaw;
        double relY = relativePosition.getX() * sinYaw + relativePosition.getY() * cosYaw;
        double relZ = relativePosition.getZ();

        // Calculate target velocity relative to robot (for lead calculation)
        // Assuming target is stationary, robot velocity creates relative motion
        double robotVelX = chassisSpeeds.vxMetersPerSecond;
        double robotVelY = chassisSpeeds.vyMetersPerSecond;

        // Rotate robot velocity to robot frame and negate (target appears to move opposite)
        double relVelX = -(robotVelX * cosYaw - robotVelY * sinYaw);
        double relVelY = -(robotVelX * sinYaw + robotVelY * cosYaw);
        double relVelZ = 0;

        // Calculate horizontal distance for logging
        distanceToTarget = Math.sqrt(relX * relX + relY * relY);

        // Select speed preset based on distance (THREE-SPEED SYSTEM)
        currentSpeedPreset = BallisticConstants.getSpeedPresetForDistance(distanceToTarget);
        currentLaunchVelocity = BallisticConstants.getLaunchVelocityForPreset(currentSpeedPreset);
        targetFlywheelRPM = BallisticConstants.getRPMForPreset(currentSpeedPreset);

        // Use ballistic solver with selected launch velocity
        Vec3 relPos = new Vec3(relX, relY, relZ);
        Vec3 relVel = new Vec3(relVelX, relVelY, relVelZ);

        lastSolverResult = BallisticSolver.solve(relPos, relVel, 
                currentLaunchVelocity, 
                BallisticConstants.DEFAULT_DRAG_COEFFICIENT, 
                ballisticParams);

        if (lastSolverResult.success || Double.isFinite(lastSolverResult.theta)) {
            // theta is pitch (launch angle), phi is yaw (turret angle)
            double launchAngleDeg = Math.toDegrees(lastSolverResult.theta);
            targetTurretAngleDeg = Math.toDegrees(lastSolverResult.phi);

            // Convert launch angle to hood angle using calibration offset
            targetHoodAngleDeg = BallisticConstants.launchAngleToHoodAngle(launchAngleDeg);

            // Clamp to mechanism limits
            targetHoodAngleDeg = MathUtil.clamp(targetHoodAngleDeg, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
            targetTurretAngleDeg = MathUtil.inputModulus(targetTurretAngleDeg, -180, 180);
        } else {
            // Fallback to simple calculation if solver fails
            targetTurretAngleDeg = Math.toDegrees(Math.atan2(relY, relX));
            double fallbackLaunchAngle = calculateFallbackHoodAngle(distanceToTarget, relZ);
            targetHoodAngleDeg = BallisticConstants.launchAngleToHoodAngle(fallbackLaunchAngle);
        }

        // Note: targetFlywheelRPM already set above based on speed preset

        // Apply commands
        applyCommands();

        // Log values
        logValues();
    }

    /**
     * Fallback hood angle calculation when solver fails
     */
    private double calculateFallbackHoodAngle(double distance, double height) {
        double baseAngle = Math.toDegrees(Math.atan2(height, distance));
        double dropCompensation = distance * 1.5;
        double hoodAngle = baseAngle + dropCompensation;
        return MathUtil.clamp(hoodAngle, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
    }

    /**
     * Get current speed preset
     */
    public SpeedPreset getCurrentSpeedPreset() {
        return currentSpeedPreset;
    }

    /**
     * Get current launch velocity used by solver (m/s)
     */
    public double getCurrentLaunchVelocity() {
        return currentLaunchVelocity;
    }

    /**
     * Apply calculated values to subsystems (direct calls, not Commands)
     */
    private void applyCommands() {
        turret.setPosition(Degrees.of(targetTurretAngleDeg));
        hood.setGoal(Degrees.of(targetHoodAngleDeg));
        flywheels.setVelocity(targetFlywheelRPM);
    }

    /**
     * Check if turret is aimed at target within tolerance
     */
    public boolean isTurretOnTarget() {
        double currentAngle = turret.getAngleDegrees();
        return Math.abs(currentAngle - targetTurretAngleDeg) < TURRET_TOLERANCE_DEG;
    }

    /**
     * Check if hood is at target angle within tolerance
     */
    public boolean isHoodOnTarget() {
        double currentAngle = hood.getAngle().in(Degrees);
        return Math.abs(currentAngle - targetHoodAngleDeg) < HOOD_TOLERANCE_DEG;
    }

    /**
     * Check if flywheel is at target speed
     */
    public boolean isFlywheelReady() {
        return flywheels.isAtVelocity(targetFlywheelRPM);
    }

    /**
     * Check if all systems are ready to shoot
     */
    public boolean isReadyToShoot() {
        return isTurretOnTarget() && isHoodOnTarget() && isFlywheelReady();
    }

    /**
     * Check if ballistic solution was found
     */
    public boolean hasSolution() {
        return lastSolverResult != null && lastSolverResult.success;
    }

    /**
     * Get distance to current target in meters
     */
    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    /**
     * Get predicted miss distance from solver (for diagnostics)
     */
    public double getPredictedMiss() {
        return lastSolverResult != null ? lastSolverResult.miss : Double.NaN;
    }

    /**
     * Get predicted time of flight (for diagnostics)
     */
    public double getTimeOfFlight() {
        return lastSolverResult != null ? lastSolverResult.tStar : Double.NaN;
    }

    /**
     * Log all values for debugging
     */
    private void logValues() {
        DogLog.log("ShooterController/Enabled", enabled);
        DogLog.log("ShooterController/DistanceToTarget", distanceToTarget, "Meters");
        DogLog.log("ShooterController/SpeedPreset", currentSpeedPreset.name());
        DogLog.log("ShooterController/LaunchVelocity", currentLaunchVelocity, "m/s");
        DogLog.log("ShooterController/TargetTurretAngle", targetTurretAngleDeg, "Degrees");
        DogLog.log("ShooterController/TargetHoodAngle", targetHoodAngleDeg, "Degrees");
        DogLog.log("ShooterController/TargetFlywheelRPM", targetFlywheelRPM, "RPM");
        DogLog.log("ShooterController/TurretOnTarget", isTurretOnTarget());
        DogLog.log("ShooterController/HoodOnTarget", isHoodOnTarget());
        DogLog.log("ShooterController/FlywheelReady", isFlywheelReady());
        DogLog.log("ShooterController/ReadyToShoot", isReadyToShoot());

        // Ballistic solver diagnostics
        DogLog.log("ShooterController/SolverSuccess", hasSolution());
        DogLog.log("ShooterController/PredictedMiss", getPredictedMiss(), "Meters");
        DogLog.log("ShooterController/TimeOfFlight", getTimeOfFlight(), "Seconds");
        if (lastSolverResult != null) {
            DogLog.log("ShooterController/SolverIterations", lastSolverResult.iterations);
        }
    }
}
