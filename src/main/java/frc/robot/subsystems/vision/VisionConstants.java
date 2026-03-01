package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String LEFT_BACK_NAME = "LeftCameraBack";
    public static final String LEFT_LEFT_NAME = "LeftCamera";
    public static final String RIGHT_BACK_NAME = "RightCameraBack";
    public static final String RIGHT_RIGHT_NAME = "RightCamera";

    public static final Transform3d LEFT_BACK_TRANSFORM = new Transform3d(
            new Translation3d(-0.3211576, 0.29718, 0.2022856),
            new Rotation3d(0, -Math.PI / 6, Units.degreesToRadians(160)));
    public static final Transform3d LEFT_LEFT_TRANSFORM = new Transform3d(
            new Translation3d(-0.263398, 0.324104, 0.2022856),
            new Rotation3d(0, -Math.PI / 6, Units.degreesToRadians(93)));

    public static final Transform3d RIGHT_BACK_TRANSFORM = new Transform3d(
            new Translation3d(-0.3387, -0.2801, 0.277),
            new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-160)));
    public static final Transform3d RIGHT_RIGHT_TRANSFORM = new Transform3d(
            new Translation3d(-0.2782, -0.3387, 0.277),
            new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-110)));
}
