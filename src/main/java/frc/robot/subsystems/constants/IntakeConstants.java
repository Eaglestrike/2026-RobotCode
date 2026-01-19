package frc.robot.subsystems.constants;

public final class IntakeConstants {
  private IntakeConstants() {
  }

  public static final class Rollers {
    public static final int SIDE_MOTOR_ID = 40;
    public static final int TOP_MOTOR_ID = 41;

    public static final boolean SIDE_INVERTED = false;
    public static final boolean TOP_INVERTED = true;

    public static final double MAX_VOLTS_SIDE = 12.0;
    public static final double MAX_VOLTS_TOP = 12.0;

    public static final double IN_VOLTS_SIDE = 9.0;
    public static final double IN_VOLTS_TOP = 9.0;

    public static final double OUT_VOLTS_SIDE = -7.0;
    public static final double OUT_VOLTS_TOP = -7.0;

    public static final double EJECT_VOLTS_SIDE = -10.0;
    public static final double EJECT_VOLTS_TOP = -10.0;

    private Rollers() {
    }
  }

  public static final class Wrist {
    public static final int MOTOR_ID = 42;
    public static final boolean INVERTED = false;

    public static final double MAX_VEL_RAD_PER_S = Math.PI;
    public static final double MAX_ACC_RAD_PER_S2 = 2.0 * Math.PI;

    public static final double GEAR_RATIO = 100.0;
    public static final double ENCODER_OFFSET_RAD = 0.0;

    public static final double MIN_ANGLE_RAD = -Math.PI / 2.0;
    public static final double MAX_ANGLE_RAD = Math.PI / 2.0;

    public static final double MAX_VOLTS = 12.0;
    public static final double AT_TARGET_TOL_RAD = Math.toRadians(2.0);

    public static final double I_ZONE_RAD = Math.toRadians(5.0);
    public static final double I_MAX = 2.0;

    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.25;

    public static final double kG = 0.8;
    public static final double kGPower = 1.0;
    public static final double kA = 0.0;
    public static final double kV = 1.0;
    public static final double kS = 0.25;

    private Wrist() {
    }
  }
}
