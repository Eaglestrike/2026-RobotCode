package frc.robot.subsystems.shooter.solver;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;

public class LUTSolver implements ShooterSolver {
    private static final int ITERS = 20;

    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    public static Translation3d getFutureTarget(Translation3d target, ChassisSpeeds fieldSpeeds, double tof) {
        double futureX = target.getX() - fieldSpeeds.vxMetersPerSecond * tof;
        double futureY = target.getY() - fieldSpeeds.vyMetersPerSecond * tof;

        return new Translation3d(futureX, futureY, target.getZ());
    }

    @Override
    public ShotData calculateShot(Pose2d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget) {
        double distance = getDistanceToTarget(launcherPose, actualTarget).in(Meters);
        Translation3d predictedTarget = actualTarget;
        double tof = TOF_MAP.get(distance);

        for (int i = 0; i < ITERS; i++) {
            tof = TOF_MAP.get(distance);
            predictedTarget = getFutureTarget(actualTarget, fieldSpeeds, tof);
            distance = getDistanceToTarget(launcherPose, predictedTarget).in(Meters);
        }

        Rotation2d turretAngle = predictedTarget.toTranslation2d()
                .minus(launcherPose.getTranslation()).getAngle()
                .minus(launcherPose.getRotation());

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
        RPM_MAP.put(0.0, 0.0);
        HOOD_MAP.put(0.0, Rotation2d.fromDegrees(0.0));
        TOF_MAP.put(0.0, 0.0);
    }
}
