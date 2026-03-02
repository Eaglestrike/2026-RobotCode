package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;
    private final StructPublisher<Pose3d> leftCameraBackPosePublisher;
    private final StructPublisher<Pose3d> leftCameraLeftPosePublisher;
    private final StructPublisher<Pose3d> rightCameraBackPosePublisher;
    private final StructPublisher<Pose3d> rightCameraRightPosePublisher;

    private final Camera[] cameras = {
            new Camera(VisionConstants.LEFT_BACK_NAME,
            VisionConstants.LEFT_BACK_TRANSFORM,
            this::addVisionMeasurement),
            new Camera(VisionConstants.LEFT_LEFT_NAME,
            VisionConstants.LEFT_LEFT_TRANSFORM,
            this::addVisionMeasurement),
            new Camera(VisionConstants.RIGHT_BACK_NAME, VisionConstants.RIGHT_BACK_TRANSFORM,
                this::addVisionMeasurement),
            new Camera(VisionConstants.RIGHT_RIGHT_NAME, VisionConstants.RIGHT_RIGHT_TRANSFORM,
                this::addVisionMeasurement)
    };

    private static final int POSE_PUBLISH_PERIOD = 10;
    private int posePublishCounter = 0;

    /** Max vision observations injected into the pose estimator per cycle (overloop mitigation). */
    private static final int MAX_OBSERVATIONS_PER_CYCLE = 2;
    private int observationsThisCycle = 0;

    /** Round-robin: only one camera's periodic runs per cycle (overloop mitigation). */
    private int cameraIndex = 0;

    public Vision(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        leftCameraBackPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("LeftCameraBackPose", Pose3d.struct)
                .publish();
        leftCameraLeftPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("LeftCameraLeftPose", Pose3d.struct)
                .publish();
        rightCameraBackPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("RightCameraBackPose", Pose3d.struct)
                .publish();
        rightCameraRightPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("RightCameraRightPose", Pose3d.struct)
                .publish();
    }

    @Override
    public void periodic() {
        observationsThisCycle = 0;

        if (++posePublishCounter >= POSE_PUBLISH_PERIOD) {
            posePublishCounter = 0;
            Pose3d robot3d = new Pose3d(drive.getState().Pose);
            leftCameraBackPosePublisher.set(robot3d.transformBy(VisionConstants.LEFT_BACK_TRANSFORM));
            leftCameraLeftPosePublisher.set(robot3d.transformBy(VisionConstants.LEFT_LEFT_TRANSFORM));
            rightCameraBackPosePublisher.set(robot3d.transformBy(VisionConstants.RIGHT_BACK_TRANSFORM));
            rightCameraRightPosePublisher.set(robot3d.transformBy(VisionConstants.RIGHT_RIGHT_TRANSFORM));
        }

        cameras[cameraIndex++ % cameras.length].periodic();
    }

    private void addVisionMeasurement(PoseObservation observation) {
        if (observationsThisCycle >= MAX_OBSERVATIONS_PER_CYCLE) {
            return;
        }
        observationsThisCycle++;
        drive.addVisionMeasurement(observation.pose(), observation.timestamp());
    }
}
