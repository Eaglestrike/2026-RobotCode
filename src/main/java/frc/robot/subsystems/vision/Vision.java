package frc.robot.subsystems.vision;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.framework.RobotBase;
import org.wpilib.command2.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;

    private final Camera[] cameras = {
            new Camera(VisionConstants.LEFT_BACK_NAME,
                    VisionConstants.LEFT_BACK_TRANSFORM,
                    this::addVisionMeasurement),
            new Camera(VisionConstants.LEFT_LEFT_NAME,
                    VisionConstants.LEFT_LEFT_TRANSFORM,
                    this::addVisionMeasurement),
            new Camera(VisionConstants.RIGHT_BACK_NAME, VisionConstants.RIGHT_BACK_TRANSFORM,
                    this::addVisionMeasurement),
            new Camera(VisionConstants.RIGHT_RIGHT_NAME,
                    VisionConstants.RIGHT_RIGHT_TRANSFORM,
                    this::addVisionMeasurement)
    };

    public Vision(EagleSwerveDrivetrain drive) {
        this.drive = drive;

        Pose3d robot3d = new Pose3d(drive.getState().Pose);
        NetworkTableInstance.getDefault().getStructTopic("LeftCameraBackPose",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.LEFT_BACK_TRANSFORM));
        NetworkTableInstance.getDefault().getStructTopic("LeftCameraLeftPose",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.LEFT_LEFT_TRANSFORM));
        NetworkTableInstance.getDefault().getStructTopic("RightCameraBackPose",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.RIGHT_BACK_TRANSFORM));
        NetworkTableInstance.getDefault().getStructTopic("RightCameraRightPose",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.RIGHT_RIGHT_TRANSFORM));
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation())
            return;
        for (var camera : cameras) {
            camera.periodic();
        }
    }

    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.3);

    private void addVisionMeasurement(PoseObservation observation) {
        Matrix<N3, N1> stdDevs = observation.isMultiTag() ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;
        drive.addVisionMeasurement(observation.pose(), observation.timestamp(), stdDevs);
    }
}
