/*
 * Original C++ ballistic-solver:
 * MIT License
 * Copyright (c) 2026 ujinf74
 * https://github.com/ujinf74/ballistic-solver
 *
 * Java port:
 * Copyright (c) 2026 Jiarui Wu
 */

package frc.robot.subsystems.shooter.solver;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.solver.BallisticSolver.Vec3;

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
    public static final double HIGH_SPEED_RPM = 7000.0;

    /** Low speed launch velocity (m/s) — converted from flywheel RPM via quadratic fit */
    public static final double LOW_SPEED_LAUNCH_VEL = ShooterConstants.flywheelRPSToBallVel(LOW_SPEED_RPM / 60.0);

    /** Medium speed launch velocity (m/s) */
    public static final double MEDIUM_SPEED_LAUNCH_VEL = ShooterConstants.flywheelRPSToBallVel(MEDIUM_SPEED_RPM / 60.0);

    /** High speed launch velocity (m/s) */
    public static final double HIGH_SPEED_LAUNCH_VEL = ShooterConstants.flywheelRPSToBallVel(HIGH_SPEED_RPM / 60.0);

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

    /** Height of shooter exit point above ground (meters) — 17.580041 in */
    public static final double SHOOTER_HEIGHT = 0.44673;


    // ================================================================
    // Solver Presets
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
