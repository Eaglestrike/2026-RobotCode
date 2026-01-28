package frc.robot.subsystems.intake;

public final class IntakeConstants {
	public static final class Rollers {
		public static final int MOTOR_ID = 41;
		public static final boolean INVERTED = false;

		public static final double MAX_VOLTS = 4.0;
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

		public static final double MAX_VOLTS = 6;

		public static final double kP = 6.0;
		public static final double kI = 0.0;
		public static final double kD = 0.25;

		public static final double kG = 0.8;
		public static final double kA = 0.0;
		public static final double kV = 1.0;
		public static final double kS = 0.25;
		public static final int STALL_LIMIT = 0;
		public static final double OFFSET = 0;
        public static final double STOW_POSITION = 0;
	}
}
