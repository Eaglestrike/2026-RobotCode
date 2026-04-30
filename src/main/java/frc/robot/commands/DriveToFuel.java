package frc.robot.commands;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Vector;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain.Fuel;

public class DriveToFuel extends Command {
    public static final double TARGET_VEL = 1.0; // In m/s
    private Queue<Pose2d> pathPositions = new LinkedList<>();
    private EagleSwerveDrivetrain drivetrain;
    Pose2d targetPose; 

    public DriveToFuel(EagleSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    private void generatePath() {
        // Step 1: Get current pose
        // Step 2: Get fuel poses from drivetrain's vision subsystem
        // Step 3: Generate path (for now, just add fuel poses to the queue). In the future, do this using fuel clustering and path generation algorithms.
        // Step 4: (Future) Change the paths to ensure they do not go through any obstacles 
        // Step 5: Add the target fuel poses to the pathPositions queue
        Pose2d currentPose = drivetrain.getState().Pose;
        pathPositions.add(currentPose); // Add current pose as the starting point of the path
        for(Fuel fuelPose : drivetrain.fuelPositions) {
            pathPositions.add(fuelPose.getPose());
        }
    }

    @Override
    public void initialize() {
        generatePath();
        DogLog.log("DriveToFuel/Path Positions", pathPositions.toArray(new Pose2d[0]));
        targetPose = pathPositions.poll(); 
    }

    @Override
    public void execute() {
        Vector<Double> targetVelocity = new Vector<>();
        if(pathPositions.isEmpty()) {
            targetVelocity.add(0.0);
            targetVelocity.add(0.0);
        } else {
            Pose2d nextPose = pathPositions.peek();
            double angleToNextPose = targetPose.getTranslation().minus(nextPose.getTranslation()).getAngle().getRadians();
            targetVelocity.add(TARGET_VEL * Math.cos(angleToNextPose));
            targetVelocity.add(TARGET_VEL * Math.sin(angleToNextPose));
        }

        drivetrain.alignPID(targetPose, targetVelocity);

        if(drivetrain.isAligned(targetPose) && !pathPositions.isEmpty())
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