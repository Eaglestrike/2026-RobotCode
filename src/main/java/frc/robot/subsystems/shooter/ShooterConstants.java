package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

public final class ShooterConstants {
    /**
     * Offset of the turret center, relative to the robot's center point
     */
    public static final Translation2d OFFSET = new Translation2d();
    public static final double MAX_VELOCITY = 0.0;
    public static final double IDLE_VELOCITY = 0;

    public static final int FLYWHEEL_MOTOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 1;
    public static final int HOOD_MOTOR_ID = 20;
}
