package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private Consumer<PoseObservation> addVisionMeasurement;

    Camera(String initialName, Transform3d robotToCamera, Consumer<PoseObservation> addVisionMeasurement) {
        camera = new PhotonCamera(initialName);
        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        this.addVisionMeasurement = addVisionMeasurement;
    }

    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return;
        }

        for (PhotonPipelineResult result : results) {
            var poseOpt = poseEstimator.update(result);
            if (poseOpt.isPresent()) {
                var pose = poseOpt.get();
                if (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                        || (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                                && pose.targetsUsed.get(0).getPoseAmbiguity() < 0.05
                                && pose.targetsUsed.get(0).getArea() > 0.25)) {
                    addVisionMeasurement
                            .accept(new PoseObservation(result.getTimestampSeconds(), pose.estimatedPose.toPose2d()));
                }
            }
        }
    }
}
