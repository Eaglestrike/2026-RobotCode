package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;

public final class ShooterConstants {

    /** Maps ball velocity (in meters per second) to flywheel velocity (in rotations per second) */
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
        public static final int MOTOR_ID = 0;
        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 0.0;
        /** In rotations per second (for when not shooting) */
        public static final double IDLE_VELOCITY = 0.0;

        public static final double GEAR_RATIO = 10/9.0;

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

        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_OUTPUT = 1.0;

        /** In RPM per second squared */
        public static final double MAX_ACCEL = 10000.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 1;

        /** Offset of the turret center, relative to the robot's center point */
        public static final Translation2d OFFSET = new Translation2d();

        /** In rotations of the turret */
        public static final double ENCODER_OFFSET = 0.0;

        /** In rotations of the turret */
        public static final double STOW_POSITION = 0.0;

        public static final double GEAR_RATIO = 135.0 / 15.0;

        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second squared */
        public static final double MAX_ACCEL = 20;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 20;

        public static final Angle ANGLE_TOLERANCE = Degree.of(2.0);

        public static final double kP = 100;
        public static final double kI = 0.0;
        public static final double kD = 5.0;
        public static final double kS = 0.0;
        public static final double kV = 1.2;
        public static final double kA = 0;

        public static final int PHOTOELECTRIC_DIO = 0;
    }

    public final class HoodConstants {
        public static final int MOTOR_ID = 0;
        public static final double GEAR_RATIO = 362.0 / 32.0;
        /** In radians */
        public final static double OFFSET = -1.36;

        public static final double MIN_ANGLE = 10;
        public static final double MAX_ANGLE = 55;

        /** In radians */
        public static final double STOW_ANGLE = 0.0;

        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second squared */
        public static final double MAX_ACCEL = 1;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 1;

        public static final Angle ANGLE_TOLERANCE = Degree.of(2.0);

        public static final double kP = 8.0;
        public static final double kI = 0.0;
        public static final double kD = 0.2;
        public static final double kS = 0.0;
        public static final double kV = 1.11;
        public static final double kA = 0.04;
        public static final double kG = 0.26;
    }

}
