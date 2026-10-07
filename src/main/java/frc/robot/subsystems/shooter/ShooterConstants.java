package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

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
        public static final double IDLE_VOLTAGE = 0.5;
        /** In rotations per minute (for when ducking and moving toward neutral zone) */
        public static final double DUCK_RPM = 1800.0;

        public static final double GEAR_RATIO = 10 / 9.0;

        /** In rotations per second */
        public static final double VELOCITY_TOLERANCE = 5.0;

        /** Current limit in Amps */
        public static final int CURRENT_LIMIT = 100;

        public static final double kP = 12;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kS = 15.4;
        public static final double kV = 0.2;
        public static final double kA = 0;

        /** In RPM per second squared */
        public static final double MAX_ACCEL = 2.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 19;
        public static final int CANCODER_ID = 18;

        /**
         * Offset of the turret center, relative to the robot's center point (meters)
         */
        public static final Translation2d OFFSET = new Translation2d(-0.12827, 0.05080);
        public static final double OFFSET_HEIGHT = 0.59673; // meters

        public static final double MAGNET_OFFSET = -0.877695;

        /** In rotations of the turret — stow at back (zero sensor position) */
        public static final double STOW_POSITION = -0.5;

        // distance from turret actual zero to turret when at expected pos -0.5
        public static final Angle ENCODER_OFFSET = Rotations.of(0.47583 + 0.5);
        // if we request zero: turret should go to 0 + 0.3281 + 0.5 (or 0 +
        // ENCODER_OFFSET)
        // if we request -0.5: turret should go to -0.5 + 0.3281 + 0.5 (or -0.5 +
        // ENCODER_OFFSET)
        public static final double ROTOR_TO_SENSOR = 9.2 * 43.0 / 87.0 * 125.0 / 36.0;
        public static final double SENSOR_TO_MECHANISM = 9.2 / ROTOR_TO_SENSOR;
        public static final double GEAR_RATIO = ROTOR_TO_SENSOR * SENSOR_TO_MECHANISM;

        public static final double MAX_VOLTS = 4;

        public static final Angle ANGLE_TOLERANCE = Degree.of(5);
        // public static final Angle FERRY_ANGLE_TOLERANCE = Degree.of(10);

        public static final double kP = 110;
        public static final double kI = 0;
        public static final double kD = 8.0;
        public static final double kS = 0.2;
        public static final double kV = 0.124 * GEAR_RATIO;
        public static final double kA = 0;

        public static final Angle FORWARD_LIMIT = Rotations.of(1 + 0.444092);
        public static final Angle BACKWARD_LIMIT = Rotations.of(0);
    }

    public final class HoodConstants {
        public static final int MOTOR_ID = 20;
        public static final double GEAR_RATIO = 362.0 / 32.0 * 20.0 / 15.0 * 48.0 / 16.0;

        /** Shooting angle at zero position (hard stop, hood most horizontal) */
        public static final double MAX_ANGLE = 90.0 - 14.894799; // 75.11°
        /** Shooting angle at max travel (hood most tilted) */
        public static final double MIN_ROT = 0.129639;

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