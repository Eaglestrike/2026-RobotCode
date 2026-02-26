package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;

public final class ShooterConstants {

    /**
     * Maps ball velocity (in meters per second) to flywheel velocity (in rotations
     * per second)
     */
    public static final InterpolatingDoubleTreeMap BALL_VEL_TO_FLYWHEEL_VEL = new InterpolatingDoubleTreeMap();
    static {
        BALL_VEL_TO_FLYWHEEL_VEL.put(1.7303, 10.0);
        BALL_VEL_TO_FLYWHEEL_VEL.put(4.7783, 20.0);
        BALL_VEL_TO_FLYWHEEL_VEL.put(6.7228, 30.0);
        BALL_VEL_TO_FLYWHEEL_VEL.put(8.0715, 40.0);
        BALL_VEL_TO_FLYWHEEL_VEL.put(9.2410, 61.0);
        BALL_VEL_TO_FLYWHEEL_VEL.put(9.7809, 62.0);
        BALL_VEL_TO_FLYWHEEL_VEL.put(10.1543, 70.0);
    }

    /**
     * Quadratic fit: flywheelRPS = 0.8593 * ballVel^2 - 3.2464 * ballVel + 13.6691
     * Inverted to: ballVel = f(flywheelRPS)
     */
    private static final double A = 0.8593;
    private static final double B = -3.2464;
    private static final double C = 13.6691;

    /**
     * Convert flywheel velocity (rps) to ball velocity (m/s)
     * using the inverted quadratic fit.
     */
    public static double flywheelRPSToBallVel(double flywheelRPS) {
        double disc = B * B - 4.0 * A * (C - flywheelRPS);
        if (disc < 0)
            return 0;
        return (-B + Math.sqrt(disc)) / (2.0 * A);
    }

    public final class FlywheelConstants {
        public static final int MOTOR_ID = 67;
        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second */
        // public static final double MAX_VELOCITY = 0.0;
        /** In rotations per second (for when not shooting) */
        public static final double IDLE_VELOCITY = 0.0;

        public static final double GEAR_RATIO = 10 / 9.0;

        /** In rotations per second */
        public static final double VELOCITY_TOLERANCE = 2.0;

        /** Current limit in Amps */
        public static final int CURRENT_LIMIT = 40;

        public static final double kP = 12.5;
        public static final double kI = 0.0;
        public static final double kD = 0.008;
        public static final double kS = 8;
        public static final double kV = 1;
        public static final double kA = 45;

        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_OUTPUT = 1.0;

        /** In RPM per second squared */
        public static final double MAX_ACCEL = 10000.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 19;

        /**
         * Offset of the turret center, relative to the robot's center point (meters)
         */
        public static final Translation2d OFFSET = new Translation2d(-0.12827, 0.05080);

        /** In rotations of the turret — zero sensor is at back, so offset by -180° */
        public static final double ENCODER_OFFSET = -0.5;

        /** In rotations of the turret — stow at back (zero sensor position) */
        public static final double STOW_POSITION = -0.5;

        public static final double GEAR_RATIO = 9.2;

        public static final double MAX_VOLTS = 0.5;

        public static final Angle ANGLE_TOLERANCE = Degree.of(0.5);

        public static final double kP = 60.0;
        public static final double kI = 15.0;
        public static final double kD = 3.0;
        public static final double kS = 0.15;
        public static final double kV = 0.0;
        public static final double kA = 0;

        public static final Angle FORWARD_LIMIT = Degree.of(130.0);
        public static final Angle BACKWARD_LIMIT = Degree.of(-230.0);

        public static final int PHOTOELECTRIC_DIO = 7;
    }

    public final class HoodConstants {
        public static final int MOTOR_ID = 20;
        public static final double GEAR_RATIO = 362.0 / 32.0 * 20.0 / 15.0 * 48.0 / 16.0;

        /** Shooting angle at zero position (hard stop, hood most horizontal) */
        public static final double MAX_ANGLE = 90.0 - 14.894799; // 75.11°
        /** Shooting angle at max travel (hood most tilted) */
        public static final double MIN_ANGLE = 90.0 - 64.0; // 26°

        /** Shooting angle at zero position (same as MAX_ANGLE) */
        public static final double OFFSET = MAX_ANGLE; // 75.11°
        /** Default goal = zero position shooting angle */
        public static final double DEFAULT_GOAL = MAX_ANGLE; // 75.11°

        public static final double STOW_ANGLE = MAX_ANGLE; // stow at hard stop

        public static final double MAX_VOLTS = 1.5;

        public static final Angle ANGLE_TOLERANCE = Degree.of(0.5);

        public static final double kP = 300.0;
        public static final double kI = 0.0;
        public static final double kD = 5.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;

    }

}