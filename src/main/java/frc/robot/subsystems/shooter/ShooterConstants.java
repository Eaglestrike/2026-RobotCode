package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

public final class ShooterConstants {

    public final class FlywheelConstants {
        public static final int MOTOR_ID = 0;
        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 0.0;
        /** In rotations per second (for when not shooting) */
        public static final double IDLE_VELOCITY = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        /** In rotations per second squared */
        public static final double TARGET_ACCEL = 0.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 1;

        /**
         * Offset of the turret center, relative to the robot's center point
         */
        public static final Translation2d OFFSET = new Translation2d();

        public static final double GEAR_RATIO = 135.0 / 15.0;

        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second squared */
        public static final double MAX_ACCEL = 20;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 20;

        public static final double kP = 100;
        public static final double kI = 0.0;
        public static final double kD = 5.0;
        public static final double kS = 0.0;
        public static final double kV = 1.2;
        public static final double kA = 0;
    }

    public final class HoodConstants {
        public static final int MOTOR_ID = 20;
        public static final double GEAR_RATIO = 362.0 / 32.0;
        /** In radians */
        public final static double OFFSET = -1.36;

        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second squared */
        public static final double MAX_ACCEL = 1;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 1;

        public static final double kP = 8.0;
        public static final double kI = 0.0;
        public static final double kD = 0.2;
        public static final double kS = 0.0;
        public static final double kV = 1.11;
        public static final double kA = 0.04;
        public static final double kG = 0.26;
    }
}
