package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public final class ShooterConstants {
    public final class FlywheelConstants {
        public static final int MOTOR_ID = 6;
        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second */
        // public static final double MAX_VELOCITY = 0.0;
        /** In rotations per minute (for when not shooting) */
        public static final double IDLE_VELOCITY = 60.0;
        /** In rotations per minute (for when ducking and moving toward neutral zone) */
        public static final double DUCK_RPM = 1800.0;

        public static final double GEAR_RATIO = 10 / 9.0;

        /** In rotations per second */
        public static final double VELOCITY_TOLERANCE = 5.0;

        /** Current limit in Amps */
        public static final int CURRENT_LIMIT = 40;

        public static final double kP = 12;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kS = 15.4;
        public static final double kV = 0.2;
        public static final double kA = 0;

        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_OUTPUT = 1.0;

        /** In RPM per second squared */
        public static final double MAX_ACCEL = 2.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 19;

        /**
         * Offset of the turret center, relative to the robot's center point (meters)
         */
        public static final Translation2d OFFSET = new Translation2d(-0.12827, 0.05080);
        public static final double OFFSET_HEIGHT = 0.59673; // meters

        /** In rotations of the turret — zero sensor is at back, so offset by -180° */
        public static final double ENCODER_OFFSET = -0.5;

        /** In rotations of the turret — stow at back (zero sensor position) */
        public static final double STOW_POSITION = -0.5;

        public static final double GEAR_RATIO = 9.2;

        public static final double MAX_VOLTS = 1.5;

        public static final Angle ANGLE_TOLERANCE = Degree.of(5);
        // public static final Angle FERRY_ANGLE_TOLERANCE = Degree.of(10);

        public static final double kP = 100;
        public static final double kI = 0;
        public static final double kD = 3.0;
        public static final double kS = 0.15;
        public static final double kV = 0.124 * GEAR_RATIO;
        public static final double kA = 0;

        public static final Angle FORWARD_LIMIT = Degree.of(130.0);
        public static final Angle BACKWARD_LIMIT = Degree.of(-230.0);
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

        public static final Angle ANGLE_TOLERANCE = Degree.of(2);

        public static final double kP = 300.0;
        public static final double kI = 0.0;
        public static final double kD = 5.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;

    }

}