package frc.robot.subsystems.climb;

public class ClimbConstants {

    // State definitions for the climb subsystem
    public enum ClimbState {
        STOWED(0),
        L1(0),
        L2(0),
        L3(0),
        MANUAL(0);

        private final double position;

        ClimbState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    };

    // Add motor IDs later
    public static final int LEFT_CLIMB_MOTOR_ID = 0;
    public static final int RIGHT_CLIMB_MOTOR_ID = 0;

    // You... probably already know what these are
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

    public final static double L1_HEIGHT = 0.0;
    public final static double L2_HEIGHT = 0.0;
    public final static double L3_HEIGHT = 0.0;
    public final static double STOW_HEIGHT = 0.0;

}
