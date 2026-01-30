package frc.robot.subsystems.climb;

public class ClimbConstants {

    // State definitions for the climb subsystem
    public enum ClimbState {
        STOWED,
        L1,
        L2,
        L3,
        MANUAL;
    };

    public static final int LEFT_CLIMB_MOTOR_ID = 0;
    public static final int RIGHT_CLIMB_MOTOR_ID = 0;

    public final static double kP = 0.0;
    public final static double kI = 0.0;
    public final static double kD = 0.0;
    public final static double kV = 0.0;
    public final static double kA = 0.0;
    public final static double kS = 0.0;
    public final static double kG = 0.0;
    public final static double LEFT_OFFSET = 0.0;
    public final static double RIGHT_OFFSET = 0.0;
    public final static double MAX_VOLTS = 4.0;
    public final static double MANUAL_RATIO = 1.0;

    public final static double L1_EXTEND_HEIGHT = 0.0;
    public final static double L2_EXTEND_HEIGHT = 0.0;
    public final static double L3_EXTEND_HEIGHT = 0.0;
    public final static double STOW_HEIGHT = 0.0;
    /** Arm position after climbing */
    public final static double CLIMBED_POSITION = 0.0;

}
