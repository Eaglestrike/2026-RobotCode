package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final String FRONT_LEFT_NAME = "CameraFrontLeft";
    public static final String FRONT_RIGHT_NAME = "CameraFrontRight";
    public static final String BACK_LEFT_NAME = "CameraBackLeft";
    public static final String BACK_RIGHT_NAME = "CameraBackRight";

    public static final Transform3d FRONT_LEFT_TRANSFORM = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d FRONT_RIGHT_TRANSFORM = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d BACK_LEFT_TRANSFORM = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final Transform3d BACK_RIGHT_TRANSFORM = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
}
