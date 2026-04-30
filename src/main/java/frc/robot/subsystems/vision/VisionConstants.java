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

    public static final Transform3d LEFT_BACK_TRANSFORM = new Transform3d(new Translation3d(-0.32519, 0.29405, 0.20228),
            new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(155)));
    public static final Transform3d LEFT_LEFT_TRANSFORM = new Transform3d(new Translation3d(-0.27895, 0.32826, 0.20228),
            new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(75)));

    public static final Transform3d RIGHT_BACK_TRANSFORM = new Transform3d(
            new Translation3d(-0.33518, -0.27948, 0.27637),
            new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-160)));
    public static final Transform3d RIGHT_RIGHT_TRANSFORM = new Transform3d(
            new Translation3d(-0.27695, -0.33518, 0.27637),
            new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-110)));
    
    public static final Transform3d LIMELIGHT_TRANSFORM = new Transform3d(
            new Translation3d(0.637437, 0.0, 0.257059),
            new Rotation3d(0, 0, 0));
}
