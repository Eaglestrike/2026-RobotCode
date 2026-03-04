package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final String LEFT_BACK_NAME = "LeftCameraBack";
    public static final String LEFT_LEFT_NAME = "LeftCamera";
    public static final String RIGHT_BACK_NAME = "RightCameraBack";
    public static final String RIGHT_RIGHT_NAME = "RightCamera";

    public static final Transform3d LEFT_BACK_TRANSFORM = new Transform3d(new Translation3d(-0.32519, 0.29405, 0.20228),
            new Rotation3d(new Quaternion(-0.49673, -0.28679, 0.40958, 0.70941)));
    public static final Transform3d LEFT_LEFT_TRANSFORM = new Transform3d(new Translation3d(-0.27895, 0.32826, 0.20228),
            new Rotation3d(new Quaternion(-0.02267, -0.01309, 0.49983, 0.86573)));
    public static final Transform3d RIGHT_BACK_TRANSFORM = new Transform3d(
            new Translation3d(-0.33518, -0.27948, 0.27637),
            new Rotation3d(new Quaternion(0.62751, 0.52654, -0.36869, -0.43939)));
    public static final Transform3d RIGHT_RIGHT_TRANSFORM = new Transform3d(
            new Translation3d(-0.27695, -0.33518, 0.27637),
            new Rotation3d(new Quaternion(0.75441, 0.63302, -0.11162, -0.13302)));
}
