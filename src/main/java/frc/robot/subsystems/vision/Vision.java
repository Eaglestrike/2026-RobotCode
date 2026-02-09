package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;

    private final Camera[] cameras = {
            // new Camera(VisionConstants.LEFT_BACK_NAME,
            // VisionConstants.LEFT_BACK_TRANSFORM,
            // this::addVisionMeasurement),
            // new Camera(VisionConstants.LEFT_LEFT_NAME,
            // VisionConstants.LEFT_LEFT_TRANSFORM,
            // this::addVisionMeasurement),
            new Camera(VisionConstants.RIGHT_BACK_NAME, VisionConstants.RIGHT_BACK_TRANSFORM,
                this::addVisionMeasurement),
            new Camera(VisionConstants.RIGHT_RIGHT_NAME, VisionConstants.RIGHT_RIGHT_TRANSFORM,
                this::addVisionMeasurement)
    };

    public Vision(EagleSwerveDrivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        Pose3d robot3d = new Pose3d(drive.getState().Pose);
        NetworkTableInstance.getDefault().getStructTopic("skibidi",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.RIGHT_BACK_TRANSFORM));
        NetworkTableInstance.getDefault().getStructTopic("skibidi toilet",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.RIGHT_RIGHT_TRANSFORM));

        for (var camera : cameras) {
            camera.periodic();
        }
    }

    private void addVisionMeasurement(PoseObservation observation) {
        drive.addVisionMeasurement(observation.pose(), observation.timestamp());
    }
}
