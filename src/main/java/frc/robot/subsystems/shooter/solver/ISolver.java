package frc.robot.subsystems.shooter.solver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ISolver {
    /**
     * Aim towards a 2d position with z @ hub funnel entrypoint
     */
    void setTarget(Translation2d target);

    State getState();

    void periodic(Pose2d pose, ChassisSpeeds speed);

    public record State(double error, Rotation2d pitch, Rotation2d azimuth, double rpm) {
    };
}
