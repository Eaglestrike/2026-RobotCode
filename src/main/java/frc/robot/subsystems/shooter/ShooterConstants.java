package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

public final class ShooterConstants {

    public final class FlywheelConstants {
        public static final int MOTOR_ID = 3;
        // public static final double MAX_VOLTS = 8.0;
        /** In RPM */
        public static final double MAX_VELOCITY = 3.0;
        /** In RPM (for when not shooting) */
        public static final double IDLE_VELOCITY = 1000.0;

        /** Current limit in Amps */
        public static final int CURRENT_LIMIT = 40;

        // NEO Vertex free speed ~6784 RPM, so kV ≈ 1/6784 ≈ 0.000147
        // 注意：feedforward (kV/kS/kA) 未配置到电机控制器（固件兼容问题），纯靠 PID
        public static final double kP = 0.0087;
        public static final double kI = 0.0;
        public static final double kD = 0.0075;
        
        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_OUTPUT = 1.0;

        /** In RPM per second */
        public static final double MAX_ACCEL = 10000.0;

        /** Tolerance for velocity check in RPM */
        public static final double VELOCITY_TOLERANCE = 100.0;
    }

    public final class TurretConstants {
        public static final int MOTOR_ID = 7;

        /**
         * Offset of the turret center, relative to the robot's center point
         */
        public static final Translation2d OFFSET = new Translation2d();

        public static final double GEAR_RATIO = 135.0 / 15.0;

        public static final double MAX_VOLTS = 8.0;
        /** In rotations per second squared */
        public static final double MAX_ACCEL = 20;
        /** In rotations per second */
        public static final double MAX_VELOCITY = 1;

        public static final double kP = 20;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kS = 0.2;
        public static final double kV = 1.2;
        public static final double kA = 0;
    }

    public final class HoodConstants {
        public static final int MOTOR_ID = 5;
        public static final boolean MOTOR_INVERTED = false;
        public static final double GEAR_RATIO = 362.0 / 32.0;
        /** In radians */
        public final static double ENCODER_OFFSET = -1.36;

        public static final double MAX_VEL = 1;
        public static final double MAX_ACC = 20;

        // Hood 机构角度（从水平面算起的仰角）
        // 射击角从垂直方向算：31.8° 从垂直 = 58.2° 仰角, 60° 从垂直 = 30° 仰角
        // Hood 角度 = 仰角 - 3.5°（膨胀偏移，见 BallisticConstants.HOOD_LAUNCH_ANGLE_OFFSET）
        public static final double MIN_ANGLE = 26.5; // degrees (最平，对应 60° 从垂直射击角)
        public static final double MAX_ANGLE = 54.7; // degrees (最陡/默认，对应 31.8° 从垂直射击角)

        public static final double MAX_VOLTS = 8.0;

        public static final double kP = 40;
        public static final double kI = 100;
        public static final double kD = 0.7;

        public static final double kG = 0; // no gravity compensation
        public static final double kA = 0.01;
        public static final double kV = 3.0;
        public static final double kS = 0.19;
        public static final double IZONE_DEG = 0.5;
        public static final double IMAX_VOLTS = 1.0;
    }
}
