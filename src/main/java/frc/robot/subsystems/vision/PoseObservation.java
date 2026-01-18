package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record PoseObservation(
        double timestamp,
        Pose2d pose,
        Matrix<N3, N1> standardDeviations) {
}
