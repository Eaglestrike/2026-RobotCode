package frc.robot.subsystems.shooter.solver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ShooterSolver {
    ShotData calculateShot(Pose2d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget);

    public record ShotData(
            Rotation2d hoodAngle,
            Rotation2d turretAngle,
            double rpm,
            Translation3d predictedTarget,
            boolean validShot) {
    };
}
