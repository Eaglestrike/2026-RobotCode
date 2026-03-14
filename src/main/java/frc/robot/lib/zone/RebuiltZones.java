package frc.robot.lib.zone;

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
            FieldConstants.RightTrench.openingTopLeft.getX() - FieldConstants.RightTrench.width / 2
                    - Units.inchesToMeters(11),
            FieldConstants.RightTrench.openingTopLeft.getX() + FieldConstants.RightTrench.width / 2
                    + Units.inchesToMeters(
                            11),
            0,
            FieldConstants.RightTrench.openingTopLeft.getY());
    public static final Zone BLUE_LEFT_TRENCH_ZONE = BLUE_RIGHT_TRENCH_ZONE.mirroredY();
    public static final Zone RED_RIGHT_TRENCH_ZONE = BLUE_RIGHT_TRENCH_ZONE.mirroredX();
    public static final Zone RED_LEFT_TRENCH_ZONE = BLUE_RIGHT_TRENCH_ZONE.mirroredX().mirroredY();

    public static final ZoneCollection TRENCH_ZONES = new ZoneCollection(
            BLUE_RIGHT_TRENCH_ZONE, BLUE_LEFT_TRENCH_ZONE, RED_RIGHT_TRENCH_ZONE, RED_LEFT_TRENCH_ZONE);

    public static final Zone SLOW_BLUE_RIGHT_TRENCH_ZONE = new Zone(
            FieldConstants.RightTrench.openingTopLeft.getX() - Units.inchesToMeters(60),
            FieldConstants.RightTrench.openingTopLeft.getX() + Units.inchesToMeters(60),
            0,
            FieldConstants.RightTrench.openingTopLeft.getY());
    public static final Zone SLOW_BLUE_LEFT_TRENCH_ZONE = SLOW_BLUE_RIGHT_TRENCH_ZONE.mirroredY();
    public static final Zone SLOW_RED_RIGHT_TRENCH_ZONE = SLOW_BLUE_RIGHT_TRENCH_ZONE.mirroredX();
    public static final Zone SLOW_RED_LEFT_TRENCH_ZONE = SLOW_BLUE_RIGHT_TRENCH_ZONE.mirroredX().mirroredY();

    public static final ZoneCollection SLOW_TRENCH_ZONES = new ZoneCollection(
            SLOW_BLUE_RIGHT_TRENCH_ZONE, SLOW_BLUE_LEFT_TRENCH_ZONE, SLOW_RED_RIGHT_TRENCH_ZONE, SLOW_RED_LEFT_TRENCH_ZONE);

    public static final Zone CENTER_ZONE = new Zone(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.oppTopCenterPoint.getX(),
            0,
            FieldConstants.fieldWidth);
    public static final Zone BLUE_ALLIANCE_ZONE = new Zone(
            0,
            FieldConstants.Hub.topCenterPoint.getX(),
            0,
            FieldConstants.fieldWidth);
    public static final Zone RED_ALLIANCE_ZONE = BLUE_ALLIANCE_ZONE.mirroredX();

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
        DogLog.log("RebuiltZones/Center Zone", CENTER_ZONE.getCorners());
        DogLog.log("RebuiltZones/Red Alliance Zone", RED_ALLIANCE_ZONE.getCorners());
        DogLog.log("RebuiltZones/Blue Alliance Zone", BLUE_ALLIANCE_ZONE.getCorners());
        DogLog.log("RebuiltZones/Slow Red Left Trench Zone",
                SLOW_RED_LEFT_TRENCH_ZONE.getCorners());
        DogLog.log("RebuiltZones/Slow Red Right Trench Zone",
                SLOW_RED_RIGHT_TRENCH_ZONE.getCorners());
        DogLog.log("RebuiltZones/Slow Blue Left Trench Zone",
                SLOW_BLUE_LEFT_TRENCH_ZONE.getCorners());
        DogLog.log("RebuiltZones/Slow Blue Right Trench Zone",
                SLOW_BLUE_RIGHT_TRENCH_ZONE.getCorners());
    }
}
