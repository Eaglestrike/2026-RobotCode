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

    public final class FlywheelConstants {
        public static final int MOTOR_ID = 67;
        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 0.0;
        /** In rotations per second (for when not shooting) */
        public static final double IDLE_VELOCITY = 0.0;

        public static final double GEAR_RATIO = 10 / 9.0;

        /** In rotations per second */
        public static final double VELOCITY_TOLERANCE = 0.5;

        /** Current limit in Amps */
        public static final int CURRENT_LIMIT = 40;

        public static final double kP = 12.5;
        public static final double kI = 0.0;
        public static final double kD = 0.008;
        public static final double kS = 8;
        public static final double kV = 1;
        public static final double kA = 45;

        /** In rotations per second squared */
        public static final double TARGET_ACCEL = 0.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 19;

        /** Offset of the turret center, relative to the robot's center point */
        public static final Translation2d OFFSET = new Translation2d();

        /** In rotations of the turret */
        public static final double ENCODER_OFFSET = 0.0;

        /** In rotations of the turret */
        public static final double STOW_POSITION = 0.0;

        public static final double GEAR_RATIO = 9.2;

        public static final double MAX_VOLTS = 2.0;

        public static final Angle ANGLE_TOLERANCE = Degree.of(2.0);

        public static final double kP = 60.0;
        public static final double kI = 0.0;
        public static final double kD = 3.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0;

        public static final double FORWARD_LIMIT = 0.5;
        public static final double BACKWARD_LIMIT = 0.0;

        public static final int PHOTOELECTRIC_DIO = 0;
    }

    public final class HoodConstants {
        public static final int MOTOR_ID = 20;
        public static final double GEAR_RATIO = 362.0 / 32.0 * 20.0 / 15.0 * 48.0 / 16.0;
        /** In radians */
        public final static double OFFSET = -0.2083;

        /** In radians */
        public static final double STOW_ANGLE = 0.0;

        public static final double MAX_VOLTS = 3.0;

        public static final Angle ANGLE_TOLERANCE = Degree.of(2.0);

        public static final double kP = 300.0;
        public static final double kI = 0.0;
        public static final double kD = 5.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kG = 0.0;

        public static final double FORWARD_LIMIT = -0.118;
        public static final double BACKWARD_LIMIT = OFFSET;
    }

}
