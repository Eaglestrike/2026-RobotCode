package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private Consumer<PoseObservation> addVisionMeasurement;

    Camera(String initialName, Transform3d robotToCamera, Consumer<PoseObservation> addVisionMeasurement) {
        camera = new PhotonCamera(initialName);
        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                robotToCamera);
        poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
        this.addVisionMeasurement = addVisionMeasurement;
    }

    public void periodic() {
        if (!camera.isConnected()) {
            return;
        }

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return;
        }
        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> poseOpt = null;
            if (result.multitagResult.isEmpty()) {
                poseOpt = poseEstimator.estimateLowestAmbiguityPose(result);
            } else {
                poseOpt = poseEstimator.estimateCoprocMultiTagPose(result);
            }
            if (poseOpt.isPresent()) {
                var pose = poseOpt.get();
                boolean isMultiTag = pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
                if (isMultiTag
                        || (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                                && pose.targetsUsed.get(0).getPoseAmbiguity() < 0.05
                                && pose.targetsUsed.get(0).getArea() > 0.25)) {
                    addVisionMeasurement
                            .accept(new PoseObservation(result.getTimestampSeconds(), pose.estimatedPose.toPose2d(),
                                    isMultiTag));
                }
            }
        }
    }
}
