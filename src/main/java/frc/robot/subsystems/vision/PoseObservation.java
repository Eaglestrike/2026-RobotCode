package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose2d;

public record PoseObservation(
        double timestamp,
        Pose2d pose,
        boolean isMultiTag) {
}
