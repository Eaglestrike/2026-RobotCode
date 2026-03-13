package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import java.time.Duration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public final class IntakeConstants {
    public static final class Rollers {
        public static final int MOTOR_ID = 34;

        public static final double VOLTS = 5.0;
        public static final double JIGGLE_VOLTS = 1.0;
        public static final double EJECT_VOLTS = -5.0;

        /*
         * Automatically ejects for AUTOEJECT_DURATION when
         * AUTOEJECT_CURRENT_THRESH of torque is applied for AUTOEJECT_TIME_THRESH
         */
        public static final Current AUTOEJECT_CURRENT_THRESH = Amps.of(30);
        public static final Time AUTOEJECT_TIME_THRESH = Seconds.of(0.5);
        public static final Time AUTOEJECT_DURATION = Seconds.of(1);
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

        public static final double UP_VOLTS = 2.0;
        public static final double DOWN_VOLTS = -1.5;

        public static final double UP_STALL_CURRENT = 15;
        public static final double DOWN_STALL_CURRENT = 5;

        public static final double RETAINING_VOLTS = 0.2;
    }
}
