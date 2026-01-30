package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum FieldConstants {
    RED_HUB(new Pose3d(11.938, 4.034536, 1.5748,
            new Rotation3d(0, 0, 0))),
    RED_OUTPOST(new Pose3d(15.75, 7.25, 0, new Rotation3d(0, 0, 0))),
    RED_FAR_SIDE(new Pose3d(15.75, 0.75, 0, new Rotation3d(0, 0, 0))),
    RED_TOWER(new Pose3d(0, 0, 0,
            new Rotation3d(0, 0, 0))), // Placeholder

    BLUE_HUB(new Pose3d(4.5974, 4.034536, 1.5748,
            new Rotation3d(0, 0, 0))),
    BLUE_OUTPOST(new Pose3d(0.75, 0.75, 0, new Rotation3d(0, 0, 0))),
    BLUE_FAR_SIDE(new Pose3d(0.75, 7.25, 0, new Rotation3d(0, 0, 0))),
    BLUE_TOWER(new Pose3d(0, 0, 0,
            new Rotation3d(0, 0, 0))); // Placeholder

    public final Pose3d value;

    private FieldConstants(Pose3d value) {
        this.value = value;
    }

    private static final Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Red : DriverStation.getAlliance().get();
    }

    public static final Pose3d getAllianceHubPosition() {
        return getAlliance() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Pose3d getAllianceOutpostPosition() {
        return getAlliance() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Pose3d getAllianceFarSidePosition() {
        return getAlliance() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value
                : BLUE_FAR_SIDE.value;
    }

    /**
     * Get the alliance tower align position to climb (for L1)
     * @param left <code>true</code> for left tower, <code>false</code> for right tower
     * @return
     */
    public static final Pose3d getAllianceTowerPosition(boolean left) {
        return getAlliance() == DriverStation.Alliance.Red ? RED_TOWER.value : BLUE_TOWER.value; // Placeholder - update logic
    }

    /**
     * @return the middle Y coordinate of the field in meters
     */
    public static final double getYMiddle() {
        return 4.034;
    }
}
