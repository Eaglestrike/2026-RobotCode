package frc.robot.subsystems.serialization;

public class SerializationConstants {
    public static final int FEEDER_MOTOR_ID = 3;
    public static final double FEEDER_MAX_VOLTS = -6;
    /** Feeder speed in rotations per second */
    public static final double FEEDER_SPEED = 30;

    /** Current limit in Amps */
    public static final int CURRENT_LIMIT = 20;

    // TODO: tune these PID values
    public static final double kP = 12.5;
    public static final double kI = 0.0;
    public static final double kD = 0.008;
    public static final double kS = 8;
    public static final double kV = 1;
    public static final double kA = 45;


    public static final int HOPPER_MOTOR_ID = 4;
    public static final double HOPPER_MAX_VOLTS = -3;
    public static final double HOPPER_NORMAL_VOLTS = -0.3;
}
