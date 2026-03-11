package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
    public static final class Rollers {
        public static final int MOTOR_ID = 16;
        public static final boolean INVERTED = false;

        public static final double VOLTS = 5.0;
        public static final double JIGGLE_VOLTS = 1.0;
    }

    public static final class Wrist {
        // FIXME: i'm intentionally commenting this out because the all the constants
        // below are wrong. PLEASE FIX THEM!!!! -max
        public static final int MOTOR_ID = 36;

        public static final double MAX_VEL = 14;
        public static final double MAX_ACC = 20;

        public static final double GEAR_RATIO = 45;
        /** In radians of the intake */
        public static final double ENCODER_OFFSET = -Math.PI / 2;

        public static final double MIN_ANGLE = Math.toDegrees(ENCODER_OFFSET); // degrees
        public static final double MAX_ANGLE = 0; // degrees
        public static final double STOW_ANGLE = MIN_ANGLE; // degrees

        public static final double MAX_VOLTS = 8.0;

        public static final double kP = 40;
        public static final double kI = 100;
        public static final double kD = 0.7;

        public static final double kG = 0.05;
        public static final double kA = 0.01;
        public static final double kV = 3.0;
        public static final double kS = 0.19;

        public static final double EJECT_ANGLE = 0;

        public static final double JIGGLE_FREQ_SEC = 1.5;

        public static final double UP_VOLTS = -2.0;
        public static final double DOWN_VOLTS = 1.5;

        public static final double UP_STALL_CURRENT = 15;
        public static final double DOWN_STALL_CURRENT = 8;

        public static final double RETAINING_VOLTS = 0.5;
    }
}
