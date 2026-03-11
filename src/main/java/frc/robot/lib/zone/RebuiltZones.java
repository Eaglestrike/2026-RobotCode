package frc.robot.lib.zone;

import dev.doglog.DogLog;
import frc.robot.FieldConstants;

public class RebuiltZones {
    public static final Zone BLUE_TOWER_ZONE = new Zone(
            FieldConstants.Tower.leftUpright.getX(),
            FieldConstants.Tower.leftUpright.getX() - FieldConstants.Tower.width,
            FieldConstants.Tower.leftUpright.getY(),
            FieldConstants.Tower.rightUpright.getY()
    );
    public static final Zone RED_TOWER_ZONE = BLUE_TOWER_ZONE.mirroredX().mirroredY();

    public static final ZoneCollection TOWER_ZONES = new ZoneCollection(BLUE_TOWER_ZONE, RED_TOWER_ZONE);

    public static void logAllZones() {
        DogLog.log("RebuiltZones/Blue Tower Zone", BLUE_TOWER_ZONE.getCorners());
        DogLog.log("RebuiltZones/Red Tower Zone", RED_TOWER_ZONE.getCorners());
    }
}
