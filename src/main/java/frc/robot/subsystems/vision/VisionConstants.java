package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String LEFT_BACK_NAME = "LeftCameraBack";
    public static final String LEFT_LEFT_NAME = "LeftCameraLeft";
    public static final String RIGHT_BACK_NAME = "RightCameraBack";
    public static final String RIGHT_RIGHT_NAME = "RightCameraRight";

    public static final Transform3d LEFT_BACK_TRANSFORM = new Transform3d(new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d LEFT_LEFT_TRANSFORM = new Transform3d(new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d RIGHT_BACK_TRANSFORM = new Transform3d(
            new Translation3d(-0.317, -0.238 - 0.038, 0.189),
            new Rotation3d(0, Units.degreesToRadians(30), 0));
    public static final Transform3d RIGHT_RIGHT_TRANSFORM = new Transform3d(
            new Translation3d(-0.276, -0.278 - 0.038, 0.189),
            new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(90)));
}
