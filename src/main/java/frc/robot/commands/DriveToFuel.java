package frc.robot.commands;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Vector;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain.Fuel;

public class DriveToFuel extends Command {
    public static final double TARGET_VEL = 0.5; // In m/s
    public static final double CLUSTER_RADIUS = 0.30; // In meters
    private Queue<Pose2d> pathPositions = new LinkedList<>();
    private EagleSwerveDrivetrain drivetrain;
    Pose2d targetPose;

    private int tickCounter = 0;

    public DriveToFuel(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    private void generatePath() {
        List<List<Pose2d>> clusters = new ArrayList<>();

        for (Fuel fuel : drivetrain.getFuelPositions()) {
            Pose2d fp = fuel.getPose();
            boolean clustered = false;
            for (List<Pose2d> cluster : clusters) {
                if (cluster.get(0).getTranslation().getDistance(fp.getTranslation()) <= CLUSTER_RADIUS) {
                    cluster.add(fp);
                    clustered = true;
                    break;
                }
            }
            if (!clustered) {
                List<Pose2d> newCluster = new ArrayList<>();
                newCluster.add(fp);
                clusters.add(newCluster);
            }
        }

        pathPositions.clear();
        // Pick the furthest point in each cluster
        Pose2d robotPose = drivetrain.getState().Pose;
        for (List<Pose2d> cluster : clusters) {
            Pose2d furthest = null;
            double maxDist = -1.0;
            for (Pose2d p : cluster) {
                double d = robotPose.getTranslation().getDistance(p.getTranslation());
                if (furthest == null || d > maxDist) {
                    furthest = p;
                    maxDist = d;
                }
            }
            if (furthest != null) {
                pathPositions.add(furthest);
            }
        }

        DogLog.log("DriveToFuel/NumClusters", clusters.size());
    }

    @Override
    public void initialize() {
        tickCounter = 0;
        generatePath();
        DogLog.log("DriveToFuel/Path Positions", pathPositions.toArray(new Pose2d[0]));
        targetPose = pathPositions.poll();
    }

    @Override
    public void execute() {
        tickCounter++;
        if (tickCounter % 25 == 0) { // Update occasionally (e.g., every 500ms at 50Hz)
            generatePath();
            DogLog.log("DriveToFuel/Path Positions", pathPositions.toArray(new Pose2d[0]));
            if (!pathPositions.isEmpty()) {
                targetPose = pathPositions.poll();
            }
        }

        Vector<Double> targetVelocity = new Vector<>();
        if (pathPositions.isEmpty()) {
            targetVelocity.add(0.0);
            targetVelocity.add(0.0);
        } else {
            Pose2d nextPose = pathPositions.peek();
            double angleToNextPose = targetPose.getTranslation().minus(nextPose.getTranslation()).getAngle()
                    .getRadians();
            targetVelocity.add(TARGET_VEL * Math.cos(angleToNextPose));
            targetVelocity.add(TARGET_VEL * Math.sin(angleToNextPose));
        }

        drivetrain.alignPID(targetPose, targetVelocity);

        if (drivetrain.isAligned(targetPose) && !pathPositions.isEmpty())
            targetPose = pathPositions.remove();
    }

    @Override
    public boolean isFinished() {
        return pathPositions.isEmpty() && targetPose != null && drivetrain.isAligned(targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        targetPose = null;
        pathPositions.clear();
    }
}