/*
 * Original C++ ballistic-solver:
 * MIT License
 * Copyright (c) 2026 ujinf74
 * https://github.com/ujinf74/ballistic-solver
 *
 * Java port:
 * Copyright (c) 2026 Jiarui Wu
 */

package frc.robot.subsystems.shooter.ballistics;

import frc.robot.subsystems.shooter.ballistics.BallisticSolver.Vec3;

/**
 * Constants and parameters for the ballistic solver
 * 
 * FRC 2026 Game Piece Specifications:
 *   - Diameter: 5.91 inches (15 cm)
 *   - Weight: 0.8 lbs (363 g)
 *   - Material: High-density foam (yellow, with FIRST/AndyMark logos)
 */
public class BallisticConstants {

    // ================================================================
    // Physical constants
    // ================================================================

    /** Gravity acceleration (m/s^2) */
    public double g = 9.80665;

    /** Wind vector (m/s) - positive X is forward, positive Y is left, positive Z is up */
    public Vec3 wind = new Vec3();

    // ================================================================
    // FRC 2026 Game Piece Parameters
    // ================================================================

    /** Game piece diameter (meters) - 5.91 inches */
    public static final double GAME_PIECE_DIAMETER = 0.15;

    /** Game piece radius (meters) */
    public static final double GAME_PIECE_RADIUS = GAME_PIECE_DIAMETER / 2.0;

    /** Game piece cross-sectional area (m²) = π × r² */
    public static final double GAME_PIECE_AREA = Math.PI * GAME_PIECE_RADIUS * GAME_PIECE_RADIUS;

    /** Game piece mass (kg) - 0.8 lbs ≈ 363g */
    public static final double GAME_PIECE_MASS = 0.363;

    /** Drag coefficient for foam ball (slightly higher than smooth sphere due to surface texture) */
    public static final double GAME_PIECE_CD = 0.50;

    /** Air density at sea level (kg/m³) */
    public static final double AIR_DENSITY = 1.225;

    // ================================================================
    // Three-Speed Shooter Configuration
    // ================================================================

    /**
     * Speed presets for the shooter (LOW / MEDIUM / HIGH)
     */
    public enum SpeedPreset {
        LOW,
        MEDIUM,
        HIGH
    }

    /** Low speed flywheel RPM - for close range shots */
    public static final double LOW_SPEED_RPM = 3000.0;

    /** Medium speed flywheel RPM - for mid range shots */
    public static final double MEDIUM_SPEED_RPM = 4500.0;

    /** High speed flywheel RPM - for long range shots */
    public static final double HIGH_SPEED_RPM = 6000.0;

    /** Low speed launch velocity (m/s) - TODO: calibrate */
    public static final double LOW_SPEED_LAUNCH_VEL = 12.0;

    /** Medium speed launch velocity (m/s) - TODO: calibrate */
    public static final double MEDIUM_SPEED_LAUNCH_VEL = 18.0;

    /** High speed launch velocity (m/s) - TODO: calibrate */
    public static final double HIGH_SPEED_LAUNCH_VEL = 24.0;

    /** Distance threshold for LOW -> MEDIUM transition (meters) */
    public static final double LOW_TO_MEDIUM_DISTANCE = 4.0;

    /** Distance threshold for MEDIUM -> HIGH transition (meters) */
    public static final double MEDIUM_TO_HIGH_DISTANCE = 7.0;

    /**
     * Get the appropriate speed preset based on distance to target
     * 
     * @param distance Distance to target in meters
     * @return The recommended speed preset
     */
    public static SpeedPreset getSpeedPresetForDistance(double distance) {
        if (distance < LOW_TO_MEDIUM_DISTANCE) {
            return SpeedPreset.LOW;
        } else if (distance < MEDIUM_TO_HIGH_DISTANCE) {
            return SpeedPreset.MEDIUM;
        } else {
            return SpeedPreset.HIGH;
        }
    }

    /**
     * Get flywheel RPM for a speed preset
     * 
     * @param preset The speed preset
     * @return Flywheel RPM
     */
    public static double getRPMForPreset(SpeedPreset preset) {
        switch (preset) {
            case LOW:
                return LOW_SPEED_RPM;
            case MEDIUM:
                return MEDIUM_SPEED_RPM;
            case HIGH:
                return HIGH_SPEED_RPM;
            default:
                return MEDIUM_SPEED_RPM;
        }
    }

    /**
     * Get launch velocity for a speed preset (for ballistic solver)
     * 
     * @param preset The speed preset
     * @return Launch velocity in m/s
     */
    public static double getLaunchVelocityForPreset(SpeedPreset preset) {
        switch (preset) {
            case LOW:
                return LOW_SPEED_LAUNCH_VEL;
            case MEDIUM:
                return MEDIUM_SPEED_LAUNCH_VEL;
            case HIGH:
                return HIGH_SPEED_LAUNCH_VEL;
            default:
                return MEDIUM_SPEED_LAUNCH_VEL;
        }
    }

    // ================================================================
    // Projectile parameters
    // ================================================================

    /** Default launch speed for solver (m/s) - uses MEDIUM preset */
    public static final double DEFAULT_LAUNCH_SPEED = MEDIUM_SPEED_LAUNCH_VEL;

    /** 
     * Quadratic drag coefficient for FRC 2026 game piece
     * 
     * kDrag = 0.5 * Cd * rho * A / m
     * 
     * Calculation:
     *   Cd = 0.50 (foam ball, rough surface)
     *   rho = 1.225 kg/m³
     *   A = π × (0.075)² = 0.01767 m²
     *   m = 0.363 kg
     * 
     *   kDrag = 0.5 × 0.50 × 1.225 × 0.01767 / 0.363
     *         = 0.0149 ≈ 0.015
     */
    public static final double DEFAULT_DRAG_COEFFICIENT = 0.015;

    // ================================================================
    // Solver parameters
    // ================================================================

    /** Use high arc trajectory (true) or low arc (false) */
    public boolean highArc = false;

    /** RK4 integration time step (seconds) */
    public double dt = 0.005;

    /** Maximum simulation time (seconds) */
    public double tMax = 3.0;

    /** Miss tolerance for success (meters) */
    public double tolMiss = 0.05;

    /** Target offset correction factor */
    public double beta = 1.0;

    /** Maximum solver iterations */
    public int maxIter = 15;

    /** Minimum pitch angle (radians) */
    public double thetaMin = -Math.PI / 2.0;

    /** Maximum pitch angle (radians) */
    public double thetaMax = Math.PI / 2.0;

    // ================================================================
    // Shooter geometry (MEASURE THESE)
    // ================================================================

    /** Height of shooter exit point above ground (meters) */
    public static final double SHOOTER_HEIGHT = 0.5;

    /** Forward offset of shooter from robot center (meters) */
    public static final double SHOOTER_FORWARD_OFFSET = 0.0;

    /** Lateral offset of shooter from robot center (meters) */
    public static final double SHOOTER_LATERAL_OFFSET = 0.0;

    // ================================================================
    // Hood Angle Calibration (Hood Shooter Geometry)
    // ================================================================
    //
    // Hood 角度定义:
    //   Hood = 0°   → 水平发射 (发射角 0°)
    //   Hood = -90° → 向下 90° (或机构限位)
    //
    // Hood Shooter 几何参数:
    //   - 底部主轮直径: 4 inch
    //   - 顶部出口滚轮: 2 inch
    //   - 顶部压缩量: 0.95 inch
    //   - 球直径: 5.91 inch (FRC 2026 Game Piece)
    //
    // 物理间隙 Gap = 5.91 - 0.95 = 4.96 inch
    // 两轮圆心距离 L = 2" + 4.96" + 1" = 7.96"
    //
    // 发射角与Hood角度的映射关系:
    //   发射角 = hood角度 + 3.5° (膨胀效应)
    //   hood角度 = 发射角 - 3.5°
    //
    // 例如:
    //   要发射 45° → hood角度 = 45° - 3.5° = 41.5°
    //   要发射 30° → hood角度 = 30° - 3.5° = 26.5°

    /**
     * Hood角度到发射角的偏移量 (degrees)
     * 
     * Hood 0° = 水平发射，偏移量仅为膨胀效应 3.5°
     * 
     * 发射角 = hood角度 + 此偏移量
     * hood角度 = 发射角 - 此偏移量
     */
    public static final double HOOD_LAUNCH_ANGLE_OFFSET = 3.5;

    /**
     * Convert launch angle (from solver) to hood angle (for mechanism)
     * 
     * @param launchAngleDeg 发射角（从水平面算起，弹道求解器输出）
     * @return hood 角度（encoder 目标值）
     */
    public static double launchAngleToHoodAngle(double launchAngleDeg) {
        return launchAngleDeg - HOOD_LAUNCH_ANGLE_OFFSET;
    }

    /**
     * Convert hood angle (from encoder) to launch angle
     * 
     * @param hoodAngleDeg hood 角度（encoder 读数）
     * @return 发射角（从水平面算起）
     */
    public static double hoodAngleToLaunchAngle(double hoodAngleDeg) {
        return hoodAngleDeg + HOOD_LAUNCH_ANGLE_OFFSET;
    }

    // ================================================================
    // Flywheel RPM ↔ Launch Speed Mapping (Experimental Data)
    // ================================================================
    // 
    // HOW TO CALIBRATE:
    // 1. Set flywheel to known RPM (e.g., 3000, 4000, 5000, 6000)
    // 2. Shoot ball horizontally from known height
    // 3. Measure horizontal distance traveled
    // 4. Calculate initial velocity: v = distance / sqrt(2 * height / g)
    // 5. Add data points below
    //
    // Example measurement setup:
    //   - Shooter height: 1.0m above ground
    //   - Ball lands at 4.5m horizontal distance
    //   - Time to fall: t = sqrt(2 * 1.0 / 9.81) = 0.452s
    //   - Initial velocity: v = 4.5 / 0.452 = 9.96 m/s

    /** 
     * Calibration data points: {RPM, LaunchSpeed (m/s)}
     * TODO: Replace with actual measured data from your shooter
     */
    private static final double[][] CALIBRATION_DATA = {
        // {RPM, Launch Speed (m/s)}
        {2000, 8.0},    // placeholder - measure this
        {3000, 12.0},   // placeholder - measure this
        {4000, 16.0},   // placeholder - measure this
        {5000, 20.0},   // placeholder - measure this
        {6000, 24.0},   // placeholder - measure this
    };

    /**
     * Convert flywheel RPM to launch speed using calibration data
     * Uses linear interpolation between data points
     * 
     * @param rpm Flywheel RPM
     * @return Estimated launch speed in m/s
     */
    public static double rpmToLaunchSpeed(double rpm) {
        // Clamp to data range
        if (rpm <= CALIBRATION_DATA[0][0]) {
            return CALIBRATION_DATA[0][1];
        }
        if (rpm >= CALIBRATION_DATA[CALIBRATION_DATA.length - 1][0]) {
            return CALIBRATION_DATA[CALIBRATION_DATA.length - 1][1];
        }

        // Find interpolation interval
        for (int i = 0; i < CALIBRATION_DATA.length - 1; i++) {
            double rpm0 = CALIBRATION_DATA[i][0];
            double rpm1 = CALIBRATION_DATA[i + 1][0];

            if (rpm >= rpm0 && rpm <= rpm1) {
                double speed0 = CALIBRATION_DATA[i][1];
                double speed1 = CALIBRATION_DATA[i + 1][1];
                double t = (rpm - rpm0) / (rpm1 - rpm0);
                return speed0 + t * (speed1 - speed0);
            }
        }

        // Fallback (shouldn't reach here)
        return 15.0;
    }

    /**
     * Convert launch speed to required flywheel RPM using calibration data
     * Uses linear interpolation between data points
     * 
     * @param launchSpeed Desired launch speed in m/s
     * @return Required flywheel RPM
     */
    public static double launchSpeedToRPM(double launchSpeed) {
        // Clamp to data range
        if (launchSpeed <= CALIBRATION_DATA[0][1]) {
            return CALIBRATION_DATA[0][0];
        }
        if (launchSpeed >= CALIBRATION_DATA[CALIBRATION_DATA.length - 1][1]) {
            return CALIBRATION_DATA[CALIBRATION_DATA.length - 1][0];
        }

        // Find interpolation interval
        for (int i = 0; i < CALIBRATION_DATA.length - 1; i++) {
            double speed0 = CALIBRATION_DATA[i][1];
            double speed1 = CALIBRATION_DATA[i + 1][1];

            if (launchSpeed >= speed0 && launchSpeed <= speed1) {
                double rpm0 = CALIBRATION_DATA[i][0];
                double rpm1 = CALIBRATION_DATA[i + 1][0];
                double t = (launchSpeed - speed0) / (speed1 - speed0);
                return rpm0 + t * (rpm1 - rpm0);
            }
        }

        // Fallback
        return 4000;
    }

    // ================================================================
    // Presets
    // ================================================================

    /** Create default parameters for close-range shots */
    public static BallisticConstants closeRange() {
        BallisticConstants p = new BallisticConstants();
        p.tMax = 1.5;
        p.tolMiss = 0.03;
        p.maxIter = 10;
        return p;
    }

    /** Create default parameters for long-range shots */
    public static BallisticConstants longRange() {
        BallisticConstants p = new BallisticConstants();
        p.tMax = 5.0;
        p.tolMiss = 0.08;
        p.maxIter = 20;
        return p;
    }

    /** Create parameters for high-arc lob shots */
    public static BallisticConstants highArcShot() {
        BallisticConstants p = new BallisticConstants();
        p.highArc = true;
        p.tMax = 5.0;
        p.tolMiss = 0.1;
        p.maxIter = 20;
        return p;
    }
}
