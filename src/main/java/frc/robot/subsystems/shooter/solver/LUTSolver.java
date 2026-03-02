package frc.robot.subsystems.shooter.solver;

import static edu.wpi.first.units.Units.Meters;

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
    public ShotData calculateShot(Pose3d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget) {
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

        return new ShotData(
                HOOD_MAP.get(distance),
                turretAngle,
                RPM_MAP.get(distance),
                predictedTarget,
                true);
    }

    /**
     * Map from shooting distance to hood angle
     */
    public static final InterpolatingTreeMap<Double, Rotation2d> HOOD_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);

    /**
     * Map from shooting distance to flywheel RPM
     */
    public static final InterpolatingDoubleTreeMap RPM_MAP = new InterpolatingDoubleTreeMap();

    /**
     * Map from shooting distance to time of flight (seconds); used
     * to account for the velocity imparted by shooting on the move
     */
    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

    static {
        // max: please recalibrate TOF usings lomo camera
        // 1.937, 1s, 74º, 1500rpm
        // 3.05, 1.2s, 73º, 1800rpm
        // 5.51, 1.22s, 58º, 2050rpm

        RPM_MAP.put(1.937, 1500.0);
        HOOD_MAP.put(1.937, Rotation2d.fromDegrees(74));
        TOF_MAP.put(1.937, 1.0);

        RPM_MAP.put(3.05, 1800.0);
        HOOD_MAP.put(3.05, Rotation2d.fromDegrees(73));
        TOF_MAP.put(3.05, 1.2);

        RPM_MAP.put(5.51, 2050.0);
        HOOD_MAP.put(5.51, Rotation2d.fromDegrees(58));
        TOF_MAP.put(5.51, 1.22);
    }
}
