package frc.robot.subsystems.shooter.solver;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ShooterSolver {
    ShotData calculateShot(Pose3d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget);

    public record ShotData(
            Rotation2d hoodAngle,
            Rotation2d turretAngle,
            double rpm,
            Translation3d predictedTarget,
            boolean validShot) {

        public static final ShotData INVALID = new ShotData(
                Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0),
                0, new Translation3d(), false);
    };
}
