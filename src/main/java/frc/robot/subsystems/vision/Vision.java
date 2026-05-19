package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final EagleSwerveDrivetrain drive;

    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision/balls");
    private final DoubleArraySubscriber centroidsSub;

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

        centroidsSub = visionTable.getDoubleArrayTopic("centroids").subscribe(new double[] {});

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
        NetworkTableInstance.getDefault().getStructTopic("LimelightPose",
                Pose3d.struct)
                .publish().set(robot3d.transformBy(VisionConstants.LIMELIGHT_TRANSFORM));
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation())
            return;

        double[] centroids = centroidsSub.get();
        Pose2d robotPose = drive.getState().Pose;
        Transform3d limelightTransform = VisionConstants.LIMELIGHT_TRANSFORM;

        for (int i = 0; i < centroids.length; i += 2) {
            double x = centroids[i];
            double y = centroids[i + 1];

            // Create a camera-relative translation and apply the limelight transform
            Translation2d cameraRelativeTranslation = new Translation2d(x, y);
            Translation2d robotRelativeTranslation = cameraRelativeTranslation.plus(limelightTransform.getTranslation().toTranslation2d());

            final double EXTRA_DISTANCE_METERS = 0;
            double dirNorm = robotRelativeTranslation.getNorm();
            if (dirNorm > 1e-6) {
                robotRelativeTranslation = robotRelativeTranslation.plus(robotRelativeTranslation.times(EXTRA_DISTANCE_METERS / dirNorm));
            }

            // Rotate the translation by the robot's heading to make it field-relative
            Translation2d fieldRelativeTranslation = robotRelativeTranslation.rotateBy(robotPose.getRotation());

            // Add the robot's position to get the final field-relative position of the fuel
            Translation2d fuelPosition = robotPose.getTranslation().plus(fieldRelativeTranslation);

            drive.addFuelPosition(fuelPosition);
        }

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
