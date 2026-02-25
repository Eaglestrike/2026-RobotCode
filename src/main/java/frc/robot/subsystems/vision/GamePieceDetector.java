package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Wraps a front-facing PhotonVision camera running an Object Detection Pipeline
 * (ML model on Orange Pi 5 NPU) to detect Fuel game pieces.
 * Returns area-weighted cluster yaw so the robot aims at the densest
 * part of a ball pile, maximizing collection per pass.
 * <p>
 * In simulation, uses known Fuel positions from SimulatedArena instead of
 * PhotonCamera, enabling full testing without hardware.
 */
public class GamePieceDetector {
    private static final double MIN_CONFIDENCE = 0.5;
    private static final double SIM_MAX_RANGE = 5.0; // meters
    private static final double SIM_FOV_DEGREES = 70.0; // half-FOV each side

    private final PhotonCamera camera;
    private final int fuelClassId;
    private final boolean isSimulation;
    private Supplier<Pose2d> robotPoseSupplier;

    private static final double REAL_ESTIMATED_DIST = 2.5;

    private boolean targetPresent = false;
    private double clusterYawDegrees = 0.0;
    private int targetCount = 0;
    private List<Translation2d> detectedPositions = Collections.emptyList();

    /**
     * @param cameraName  PhotonVision camera name (must match the GUI)
     * @param fuelClassId ML class ID for Fuel (check PhotonVision Object Detection tab)
     */
    public GamePieceDetector(String cameraName, int fuelClassId) {
        this.camera = new PhotonCamera(cameraName);
        this.fuelClassId = fuelClassId;
        this.isSimulation = RobotBase.isSimulation();
    }

    /**
     * Must be called to enable simulation mode.
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        this.robotPoseSupplier = supplier;
    }

    public void periodic() {
        if (isSimulation && robotPoseSupplier != null) {
            periodicSim();
        } else {
            periodicReal();
        }

        DogLog.log("GamePiece/HasTarget", targetPresent);
        DogLog.log("GamePiece/Count", targetCount);
        if (targetPresent) {
            DogLog.log("GamePiece/ClusterYawDeg", clusterYawDegrees);
        }
    }

    private void periodicReal() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);

        double weightedYawSum = 0;
        double totalArea = 0;
        int count = 0;
        List<Translation2d> positions = new ArrayList<>();

        Pose2d robotPose = (robotPoseSupplier != null) ? robotPoseSupplier.get() : new Pose2d();

        for (PhotonTrackedTarget target : latest.getTargets()) {
            if (target.getDetectedObjectClassID() != fuelClassId) continue;
            if (target.getDetectedObjectConfidence() < MIN_CONFIDENCE) continue;

            double area = target.getArea();
            double yaw = target.getYaw();
            weightedYawSum += yaw * area;
            totalArea += area;
            count++;

            double fieldAngle = robotPose.getRotation().getRadians() + Math.toRadians(yaw);
            positions.add(new Translation2d(
                    robotPose.getX() + REAL_ESTIMATED_DIST * Math.cos(fieldAngle),
                    robotPose.getY() + REAL_ESTIMATED_DIST * Math.sin(fieldAngle)));
        }

        targetCount = count;
        targetPresent = count > 0 && totalArea > 0;
        if (targetPresent) {
            clusterYawDegrees = weightedYawSum / totalArea;
        }
        detectedPositions = positions;
    }

    private void periodicSim() {
        Pose2d robotPose = robotPoseSupplier.get();
        double robotHeading = robotPose.getRotation().getRadians();

        Pose3d[] fuelPoses = SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel");

        double weightedYawSum = 0;
        double totalArea = 0;
        int count = 0;

        List<Translation2d> positions = new ArrayList<>();

        for (Pose3d fuelPose : fuelPoses) {
            Translation2d fuelPos = fuelPose.toPose2d().getTranslation();
            Translation2d delta = fuelPos.minus(robotPose.getTranslation());
            double distance = delta.getNorm();

            if (distance > SIM_MAX_RANGE || distance < 0.1) continue;

            double fieldAngle = Math.atan2(delta.getY(), delta.getX());
            double relativeAngle = fieldAngle - robotHeading;
            relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

            double relativeAngleDeg = Math.toDegrees(relativeAngle);
            if (Math.abs(relativeAngleDeg) > SIM_FOV_DEGREES) continue;

            double simArea = 1.0 / (distance * distance);
            weightedYawSum += relativeAngleDeg * simArea;
            totalArea += simArea;
            count++;
            positions.add(fuelPos);
        }

        targetCount = count;
        targetPresent = count > 0 && totalArea > 0;
        if (targetPresent) {
            clusterYawDegrees = weightedYawSum / totalArea;
        }
        detectedPositions = positions;
    }

    public boolean hasTarget() {
        return targetPresent;
    }

    public int getTargetCount() {
        return targetCount;
    }

    /**
     * @return Area-weighted average yaw toward the densest cluster of detected
     *         Fuel balls (degrees). Closer balls (larger area) have more weight,
     *         so the robot aims at the center of the nearest/densest pile.
     *         Positive = cluster is to the left of camera center.
     */
    public OptionalDouble getClusterYawDegrees() {
        if (!targetPresent) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(clusterYawDegrees);
    }

    /**
     * @return Field-relative positions of detected balls. In simulation, these are
     *         exact positions; on real hardware, positions are estimated from camera
     *         yaw at a fixed assumed distance ({@value #REAL_ESTIMATED_DIST}m).
     */
    public List<Translation2d> getDetectedPositions() {
        return detectedPositions;
    }
}
