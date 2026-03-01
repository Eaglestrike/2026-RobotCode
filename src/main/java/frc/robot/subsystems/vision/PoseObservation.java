package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public record PoseObservation(
        double timestamp,
        Pose2d pose,
        boolean isMultiTag) {
}
