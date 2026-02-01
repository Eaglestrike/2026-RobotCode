package frc.robot.subsystems.intake;

public final class IntakeConstants {
	public static final class Rollers {
		public static final int MOTOR_ID = 14;
		public static final boolean INVERTED = false;

		public static final double MAX_VOLTS = 3.0;
	}

	public static final class Wrist {
		public static final int MOTOR_ID = 24;

		public static final double MAX_VEL = 14;
		public static final double MAX_ACC = 20;

		public static final double GEAR_RATIO = 100.0;
		public static final double ENCODER_OFFSET = -0.304283; // rotations

		public static final double MIN_ANGLE = -17.50; // degrees
		public static final double MAX_ANGLE = 114.0; // degrees
		public static final double STOW_ANGLE = 0; // degrees

		public static final double MAX_VOLTS = 12.0;

		public static final double kP = 40;
		public static final double kI = 100;
		public static final double kD = 0.7;

		public static final double kG = 0.05;
		public static final double kA = 0.01;
		public static final double kV = 3.0;
		public static final double kS = 0.19;
		public static final double IZONE_DEG = 0.5;
		public static final double IMAX_VOLTS = 1.0;
	}
}
