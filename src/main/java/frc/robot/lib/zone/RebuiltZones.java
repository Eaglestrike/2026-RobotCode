package frc.robot.lib.zone;

import static edu.wpi.first.units.Units.Inches;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class RebuiltZones {
    public static final Zone BLUE_TOWER_ZONE = new Zone(
            FieldConstants.Tower.leftUpright.getX() - FieldConstants.Tower.width,
            FieldConstants.Tower.leftUpright.getX(),
            FieldConstants.Tower.rightUpright.getY(),
            FieldConstants.Tower.leftUpright.getY());
    public static final Zone RED_TOWER_ZONE = BLUE_TOWER_ZONE.mirroredX().mirroredY();

    public static final ZoneCollection TOWER_ZONES = new ZoneCollection(BLUE_TOWER_ZONE, RED_TOWER_ZONE);

    public static final Zone BLUE_RIGHT_TRENCH_ZONE = new Zone(
            FieldConstants.RightTrench.openingTopLeft.getX() - FieldConstants.RightTrench.width,
            FieldConstants.RightTrench.openingTopLeft.getX() + FieldConstants.RightTrench.width,
            0,
            FieldConstants.RightTrench.openingTopLeft.getY());
    public static final Zone BLUE_LEFT_TRENCH_ZONE = BLUE_RIGHT_TRENCH_ZONE.mirroredY();
    public static final Zone RED_RIGHT_TRENCH_ZONE = BLUE_RIGHT_TRENCH_ZONE.mirroredX();
    public static final Zone RED_LEFT_TRENCH_ZONE = BLUE_RIGHT_TRENCH_ZONE.mirroredX().mirroredY();

    public static final ZoneCollection TRENCH_ZONES = new ZoneCollection(
            BLUE_RIGHT_TRENCH_ZONE, BLUE_LEFT_TRENCH_ZONE, RED_RIGHT_TRENCH_ZONE, RED_LEFT_TRENCH_ZONE);

    public static void logAllZones() {
        DogLog.log("RebuiltZones/Blue Tower Zone", BLUE_TOWER_ZONE.getCorners());
        DogLog.log("RebuiltZones/Red Tower Zone", RED_TOWER_ZONE.getCorners());
        DogLog.log("RebuiltZones/Red Left Trench Zone",
                RED_LEFT_TRENCH_ZONE.getCorners());
        DogLog.log("RebuiltZones/Red Right Trench Zone",
                RED_RIGHT_TRENCH_ZONE.getCorners());
        DogLog.log("RebuiltZones/Blue Left Trench Zone",
                BLUE_LEFT_TRENCH_ZONE.getCorners());
        DogLog.log("RebuiltZones/Blue Right Trench Zone",
                BLUE_RIGHT_TRENCH_ZONE.getCorners());
    }
}
