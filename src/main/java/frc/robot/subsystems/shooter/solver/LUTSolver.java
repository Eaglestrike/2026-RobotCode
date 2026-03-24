package frc.robot.subsystems.shooter.solver;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

public class LUTSolver implements ShooterSolver {
    private static final int ITERS = 20;

    public static Distance getDistanceToTarget(Translation3d launcherPose, Translation3d target) {
        return Meters.of(launcherPose.getDistance(target));
    }

    public static Translation3d getFutureTarget(Translation3d target, ChassisSpeeds fieldSpeeds, double tof) {
        double futureX = target.getX() - fieldSpeeds.vxMetersPerSecond * tof;
        double futureY = target.getY() - fieldSpeeds.vyMetersPerSecond * tof;

        return new Translation3d(futureX, futureY, target.getZ());
    }

    @Override
    public ShotData calculateShot(Pose3d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget, boolean isFerrying) {
        double distance = getDistanceToTarget(launcherPose.getTranslation(), actualTarget).in(Meters);
        Translation3d predictedTarget = actualTarget;
        double tof = TOF_MAP.get(distance);

        for (int i = 0; i < ITERS; i++) {
            tof = TOF_MAP.get(distance);
            predictedTarget = getFutureTarget(actualTarget, fieldSpeeds, tof);
            distance = getDistanceToTarget(launcherPose.getTranslation(), predictedTarget).in(Meters);
        }

        Rotation2d turretAngle = predictedTarget.toTranslation2d()
                .minus(launcherPose.getTranslation().toTranslation2d())
                .getAngle().minus(launcherPose.getRotation().toRotation2d());

        if(isFerrying) {
            return new ShotData(
                FERRY_HOOD_MAP.get(distance), 
                turretAngle, 
                FERRY_RPM_MAP.get(distance), 
                predictedTarget,
                true);
        }
        return new ShotData(
                 HOOD_MAP.get(distance),
//                getHoodAngle(distance),
                turretAngle,
                RPM_MAP.get(distance),
                predictedTarget,
                true);
    }

//    private Rotation2d getHoodAngle(double distance) {
//        final double A = -0.553677;
//        final double B = -1.46417;
//        final double C = 79.62148;
//        return Rotation2d.fromDegrees(A * distance * distance + B * distance + C);
//    }

    /**
     * Map from shooting distance to hood angle
     */
    public static final InterpolatingTreeMap<Double, Rotation2d> HOOD_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);

    public static final InterpolatingTreeMap<Double, Rotation2d> FERRY_HOOD_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);

    /**
     * Map from shooting distance to flywheel RPM
     */
    public static final InterpolatingDoubleTreeMap RPM_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap FERRY_RPM_MAP = new InterpolatingDoubleTreeMap();

    /**
     * Map from shooting distance to time of flight (seconds); used
     * to account for the velocity imparted by shooting on the move
     */
    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

    static {
        RPM_MAP.clear();
        HOOD_MAP.clear();
        TOF_MAP.clear();
        FERRY_RPM_MAP.clear();
        FERRY_HOOD_MAP.clear();

        // ----- SHOOT MAP -----
        RPM_MAP.put(1.75, 1400.0);
        HOOD_MAP.put(1.75, Rotation2d.fromDegrees(75));
        TOF_MAP.put(1.75, 4.2 * 0.125);

        RPM_MAP.put(2.37, 1500.0);
        HOOD_MAP.put(2.37, Rotation2d.fromDegrees(74));
        TOF_MAP.put(2.37, 3.9 * 0.125);

        RPM_MAP.put(2.97, 1596.0);
        HOOD_MAP.put(2.97, Rotation2d.fromDegrees(69));
        TOF_MAP.put(2.97, 4.53 * 0.125);

        RPM_MAP.put(3.31, 1650.0);
        HOOD_MAP.put(3.31, Rotation2d.fromDegrees(67));
        TOF_MAP.put(3.31, 5.0 * 0.125);

        RPM_MAP.put(3.88, 1741.0);
        HOOD_MAP.put(3.88, Rotation2d.fromDegrees(63));
        TOF_MAP.put(3.88, 5.0 * 0.125);

        RPM_MAP.put(4.29, 1806.0);
        HOOD_MAP.put(4.29, Rotation2d.fromDegrees(61));
        TOF_MAP.put(4.29, 5.16 * 0.125);

        RPM_MAP.put(5.214, 2000.0);
        HOOD_MAP.put(5.214, Rotation2d.fromDegrees(53));
        TOF_MAP.put(5.214, 5.25 * 0.125);
        // ----- END SHOOT MAP -----

        // ----- FERRY MAP -----
        FERRY_RPM_MAP.put(3.79, 1660.0);
        FERRY_HOOD_MAP.put(3.79, Rotation2d.fromDegrees(70));

        FERRY_RPM_MAP.put(6.6, 2100.0);
        FERRY_HOOD_MAP.put(6.6, Rotation2d.fromDegrees(61));

        FERRY_RPM_MAP.put(12.6, 4500.0);
        FERRY_HOOD_MAP.put(12.6, Rotation2d.fromDegrees(38));
        // ----- END FERRY MAP -----
    }

    public enum ManualPosition {
        Position1, // in front of hub
        Position2, // any trench (tuned with left)
        Position3, // any corner (tuned with right)
        Position4, // IFO tower
    }

    public static final Map<ManualPosition, Double> MANUAL_RPM_MAP = new HashMap<>();
    public static final Map<ManualPosition, Rotation2d> MANUAL_HOOD_MAP = new HashMap<>();

    static {
        MANUAL_RPM_MAP.put(ManualPosition.Position1, 1400.0);
        MANUAL_HOOD_MAP.put(ManualPosition.Position1, Rotation2d.fromDegrees(75));

        MANUAL_RPM_MAP.put(ManualPosition.Position2, 1651.0);
        MANUAL_HOOD_MAP.put(ManualPosition.Position2, Rotation2d.fromDegrees(70));

        MANUAL_RPM_MAP.put(ManualPosition.Position3, 1955.0);
        MANUAL_HOOD_MAP.put(ManualPosition.Position3, Rotation2d.fromDegrees(57));

        MANUAL_RPM_MAP.put(ManualPosition.Position4, 1596.0);
        MANUAL_HOOD_MAP.put(ManualPosition.Position4, Rotation2d.fromDegrees(69));
    }
}