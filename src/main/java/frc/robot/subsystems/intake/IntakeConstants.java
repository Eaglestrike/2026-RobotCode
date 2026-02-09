package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
	public static final class Rollers {
		public static final int MOTOR_ID = 14;
		public static final boolean INVERTED = false;

		public static final double VOLTS = 3.0;
	}

	public static final class Wrist {
		public static final int MOTOR_ID = 24;

		public static final double MAX_VEL = 14;
		public static final double MAX_ACC = 20;

		public static final double GEAR_RATIO = 100.0;
		/** In radians of the intake */
		public static final Angle ENCODER_OFFSET = Radians.of(-0.304283 - Math.PI / 2);

		public static final Angle MIN_ANGLE = ENCODER_OFFSET;
		public static final Angle MAX_ANGLE = Degrees.of(24);
		public static final Angle STOW_ANGLE = Degrees.of(-90);

		public static final double MAX_VOLTS = 8.0;

		public static final double kP = 40;
		public static final double kI = 100;
		public static final double kD = 0.7;

		public static final double kG = 0.05;
		public static final double kA = 0.01;
		public static final double kV = 3.0;
		public static final double kS = 0.19;
		public static final Angle IZONE = Degrees.of(0.5);
		public static final double IMAX_VOLTS = 1.0;

		public static final Angle EJECT_ANGLE = Degrees.of(0);

		public static final Angle TOLERANCE = Degrees.of(1.0);
	}
}
