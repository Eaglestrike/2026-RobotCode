package frc.robot.subsystems.shooter.solver;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

public interface ShooterSolver {
    ShotData calculateShot(Pose3d launcherPose, ChassisVelocities fieldVelocities, ChassisVelocities kickerVelocities,
            Translation3d actualTarget, boolean isFerrying, Pose2d drivebasePose);

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
