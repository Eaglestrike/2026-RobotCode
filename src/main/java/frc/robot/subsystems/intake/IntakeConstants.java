package frc.robot.subsystems.intake;

public final class IntakeConstants {
	public static final class Rollers {
		public static final int MOTOR_ID = 41;
		public static final boolean INVERTED = false;

		public static final double MAX_VOLTS = 4.0;
	}

	public static final class Wrist {
		public static final int MOTOR_ID = 24;
		public static final boolean INVERTED = true;

		public static final double MAX_VEL_RAD_PER_S = 1;
		public static final double MAX_ACC_RAD_PER_S2 = 1;

		public static final double GEAR_RATIO = 100.0;
		public static final double ENCODER_OFFSET_RAD = -0.304283;

		public static final double MIN_ANGLE_RAD = -Math.PI / 2.0;
		public static final double MAX_ANGLE_RAD = 0.3;

		public static final double MAX_VOLTS = 6;

		public static final double kP = 40;
		public static final double kI = 100;
		public static final double kD = 0.7;

		public static final double kG = 0.05;
		public static final double kA = 0.01;
		public static final double kV = 3.0;
		public static final double kS = 0.19;
		public static final int STALL_LIMIT = 0;
	}
}
