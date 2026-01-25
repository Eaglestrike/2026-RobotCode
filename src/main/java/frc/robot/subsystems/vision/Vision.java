package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;

    private final Camera[] cameras = {
            new Camera(VisionConstants.FRONT_LEFT_NAME, VisionConstants.FRONT_LEFT_TRANSFORM, this::addVisionMeasurement),
            new Camera(VisionConstants.FRONT_RIGHT_NAME, VisionConstants.FRONT_RIGHT_TRANSFORM, this::addVisionMeasurement),
            new Camera(VisionConstants.BACK_LEFT_NAME, VisionConstants.BACK_LEFT_TRANSFORM, this::addVisionMeasurement),
            new Camera(VisionConstants.BACK_RIGHT_NAME, VisionConstants.BACK_RIGHT_TRANSFORM, this::addVisionMeasurement)
    };

    Vision(EagleSwerveDrivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        for (var camera : cameras) {
            camera.periodic();
        }
    }

    private void addVisionMeasurement(PoseObservation observation) {
        drive.addVisionMeasurement(observation.pose(), observation.timestamp());
    }
}
