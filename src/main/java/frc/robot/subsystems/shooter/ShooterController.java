package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

/**
 * ShooterController - Unified aiming controller for Turret + Hood + Flywheel
 * 
 * Calculates and coordinates:
 * - Turret angle (horizontal aiming)
 * - Hood angle (vertical aiming based on distance)
 * - Flywheel speed (based on distance)
 */
public class ShooterController {
    private final EagleSwerveDrivetrain drive;
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;

    // Current target
    private Pose3d targetPose = null;
    private boolean enabled = false;

    // Calculated values
    private double targetTurretAngleDeg = 0;
    private double targetHoodAngleDeg = 0;
    private double targetFlywheelRPM = FlywheelConstants.IDLE_VELOCITY;
    private double distanceToTarget = 0;

    // Tolerances
    private static final double TURRET_TOLERANCE_DEG = 2.0;
    private static final double HOOD_TOLERANCE_DEG = 1.0;

    public ShooterController(EagleSwerveDrivetrain drive, ShooterTurret turret, 
                             ShooterHood hood, ShooterFlywheels flywheels) {
        this.drive = drive;
        this.turret = turret;
        this.hood = hood;
        this.flywheels = flywheels;
    }

    /**
     * Set the target to aim at
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
     * Call this periodically to update all aiming calculations and commands
     */
    public void update() {
        if (targetPose == null || !enabled) {
            // Idle mode - just maintain idle flywheel speed
            flywheels.setVelocity(FlywheelConstants.IDLE_VELOCITY);
            return;
        }

        // Get robot and turret position
        Pose2d robotPose = drive.getState().Pose;
        Translation2d turretPosition = robotPose.getTranslation()
                .plus(TurretConstants.OFFSET.rotateBy(robotPose.getRotation()));

        // Calculate distance to target (horizontal distance)
        Translation2d targetXY = targetPose.toPose2d().getTranslation();
        distanceToTarget = turretPosition.getDistance(targetXY);

        // Calculate turret angle
        targetTurretAngleDeg = calculateTurretAngle(robotPose, turretPosition, targetXY);

        // Calculate hood angle based on distance and target height
        double targetHeight = targetPose.getZ();
        targetHoodAngleDeg = calculateHoodAngle(distanceToTarget, targetHeight);

        // Calculate flywheel speed based on distance
        targetFlywheelRPM = calculateFlywheelSpeed(distanceToTarget);

        // Apply commands
        applyCommands();

        // Log values
        logValues();
    }

    /**
     * Calculate the turret angle needed to face the target
     */
    private double calculateTurretAngle(Pose2d robotPose, Translation2d turretPos, Translation2d targetPos) {
        // Vector from turret to target
        Translation2d toTarget = targetPos.minus(turretPos);
        
        // Angle in field coordinates
        Rotation2d fieldAngle = toTarget.getAngle();
        
        // Convert to robot-relative angle
        Rotation2d relativeAngle = fieldAngle.minus(robotPose.getRotation());
        
        // Normalize to [-180, 180]
        double angleDeg = MathUtil.inputModulus(relativeAngle.getDegrees(), -180, 180);
        
        // Clamp to turret limits
        angleDeg = MathUtil.clamp(angleDeg, -180, 180);
        
        return angleDeg;
    }

    /**
     * Calculate hood angle based on distance and target height
     * Uses simple projectile physics approximation
     * 
     * TODO: Replace with lookup table or more accurate ballistics model
     */
    private double calculateHoodAngle(double distance, double targetHeight) {
        // Simple linear interpolation for now
        // Close range (2m) -> steeper angle, Far range (10m) -> shallower angle
        
        // Base angle calculation using atan
        double baseAngle = Math.toDegrees(Math.atan2(targetHeight, distance));
        
        // Add compensation for projectile drop (empirical adjustment)
        // This needs tuning based on actual ball physics
        double dropCompensation = distance * 1.5; // degrees per meter, needs tuning
        
        double hoodAngle = baseAngle + dropCompensation;
        
        // Clamp to hood limits
        hoodAngle = MathUtil.clamp(hoodAngle, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        
        return hoodAngle;
    }

    /**
     * Calculate flywheel speed based on distance
     * 
     * TODO: Replace with lookup table based on testing
     */
    private double calculateFlywheelSpeed(double distance) {
        // Linear interpolation between min and max distance
        // Closer = lower speed, farther = higher speed
        
        double minDistance = 2.0; // meters
        double maxDistance = 10.0; // meters
        double minRPM = 3000;
        double maxRPM = FlywheelConstants.MAX_VELOCITY;
        
        // Linear interpolation
        double t = MathUtil.clamp((distance - minDistance) / (maxDistance - minDistance), 0, 1);
        double rpm = minRPM + t * (maxRPM - minRPM);
        
        return rpm;
    }

    /**
     * Apply calculated values to subsystems (direct calls, not Commands)
     */
    private void applyCommands() {
        // Turret
        turret.setPosition(Degrees.of(targetTurretAngleDeg));
        
        // Hood
        hood.setGoal(Degrees.of(targetHoodAngleDeg));
        
        // Flywheels
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
     * Get distance to current target in meters
     */
    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    /**
     * Log all values for debugging
     */
    private void logValues() {
        DogLog.log("ShooterController/Enabled", enabled);
        DogLog.log("ShooterController/DistanceToTarget", distanceToTarget, "Meters");
        DogLog.log("ShooterController/TargetTurretAngle", targetTurretAngleDeg, "Degrees");
        DogLog.log("ShooterController/TargetHoodAngle", targetHoodAngleDeg, "Degrees");
        DogLog.log("ShooterController/TargetFlywheelRPM", targetFlywheelRPM, "RPM");
        DogLog.log("ShooterController/TurretOnTarget", isTurretOnTarget());
        DogLog.log("ShooterController/HoodOnTarget", isHoodOnTarget());
        DogLog.log("ShooterController/FlywheelReady", isFlywheelReady());
        DogLog.log("ShooterController/ReadyToShoot", isReadyToShoot());
    }
}
