package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
    public static final class Rollers {
        public static final int MOTOR_ID = 34;
        public static final int FOLLOWER_MOTOR_ID = 30;

        public static final double VOLTS = 6.0;
        public static final double EJECT_VOLTS = -5.0;
        public static final int CURRENT_LIMIT = 40;
        public static final int STALL_THRESHOLD = 100; // in stator current amps
        public static final int VELOCITY_THRESHOLD = 10; // in motor rotations per second
    }

    public static final class Wrist {
        public static final int MOTOR_ID = 36;

        // 12t -> 60t -> 24t -> 74t -> 9t sprocket -> 24t sprocket
        public static final double GEAR_RATIO = 1 / ((12.0 / 60.0) * (24.0 / 74.0) * (9.0 / 24.0));
        public static final Angle ENCODER_OFFSET = Rotations.of(0);

        // min angle is all the way up, max angle is all the way down
        // down positive
        public static final Angle MIN_ANGLE = Rotations.of(0);
        public static final Angle MAX_ANGLE = Rotations.of(0.2);

        public static final Angle STOW_ANGLE = MIN_ANGLE;
        public static final Angle INTAKE_ANGLE = MAX_ANGLE;
        public static final Angle EJECT_ANGLE = MAX_ANGLE;

        public static final double MAX_VOLTS = 4.0;
        /*
         * pid constants:
         * kP:
         * kD:
         */
    }
}
