package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Time;

public final class IntakeConstants {
    public static final class Rollers {
        public static final int MOTOR_ID = 34;
        public static final int FOLLOWER_MOTOR_ID = 30;

        public static final double VOLTS = 6.0;
        public static final double JIGGLE_VOLTS = 2.0;
        public static final double EJECT_VOLTS = -5.0;
        public static final int CURRENT_LIMIT = 40;
        public static final int STALL_THRESHOLD = 100; // in stator current amps
        public static final int VELOCITY_THRESHOLD = 10; // in motor rotations per second
    }

    public static final class Wrist {
        public static final int MOTOR_ID = 36;

        public static final double GEAR_RATIO = 45;
        public static final double ENCODER_OFFSET = -Math.PI / 2;

        public static final double MIN_ANGLE = Math.toDegrees(ENCODER_OFFSET); // degrees
        public static final double MAX_ANGLE = 0; // degrees
        public static final double STOW_ANGLE = MIN_ANGLE; // degrees

        public static final double MAX_VOLTS = 4.0;

        public static final double EJECT_ANGLE = 0;

        public static final double JIGGLE_FREQ_SEC = 1.5;

        public static final double UP_VOLTS = 1.5;
        public static final double DOWN_VOLTS = -2.0;
        public static final double JIGGLE_DOWN_VOLTS = 0;
        public static final double JIGGLE_UP_VOLTS = 1.5;

        public static final double UP_STALL_CURRENT = 10.5;
        public static final double DOWN_STALL_CURRENT = 5;

        public static final double RETAINING_VOLTS = 0.2;

        public static final double TIMEOUT_SECONDS = 1.0; // for wrist to terminate
    }
}
