package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.vision.GamePieceDetector;

/**
 * Autonomous collect command: pathfinds to a driver-designated area while
 * rotating toward detected balls (or toward the target point as fallback).
 * Intake runs for the entire duration.
 *
 * Uses setRotationTargetOverride to dynamically control heading so that
 * PathPlanner's own PID + feedforward work together correctly.
 */
public class AutoCollectCommand extends Command {
    private static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540),
            Units.degreesToRadians(720));

    private final EagleSwerveDrivetrain drivetrain;
    private final GamePieceDetector detector;
    private final Superstructure superstructure;
    private final Pose2d targetPose;
    private final PathConstraints constraints;

    private Command pathfindCommand;
    private Command intakeCommand;
    private int executeCycles = 0;

    public AutoCollectCommand(
            EagleSwerveDrivetrain drivetrain,
            GamePieceDetector detector,
            Superstructure superstructure,
            Pose2d targetPose,
            PathConstraints constraints) {
        this.drivetrain = drivetrain;
        this.detector = detector;
        this.superstructure = superstructure;
        this.targetPose = targetPose;
        this.constraints = constraints;

        addRequirements(drivetrain);
    }

    public AutoCollectCommand(
            EagleSwerveDrivetrain drivetrain,
            GamePieceDetector detector,
            Superstructure superstructure,
            Pose2d targetPose) {
        this(drivetrain, detector, superstructure, targetPose, DEFAULT_CONSTRAINTS);
    }

    @Override
    public void initialize() {
        executeCycles = 0;
        Pose2d currentPose = drivetrain.getState().Pose;
        System.out.println("[AutoCollect] Starting! Target=(" + targetPose.getX() + ", " + targetPose.getY()
                + ") Robot=(" + currentPose.getX() + ", " + currentPose.getY() + ")");

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        Rotation2d headingToTarget = computeHeadingToTarget(currentPose);
        Pose2d poseWithHeading = new Pose2d(targetPose.getTranslation(), headingToTarget);

        pathfindCommand = AutoBuilder.pathfindToPose(poseWithHeading, constraints, 0.0);
        intakeCommand = superstructure.intake();

        pathfindCommand.initialize();
        intakeCommand.schedule();
    }

    @Override
    public void execute() {
        executeCycles++;

        if (pathfindCommand != null) {
            pathfindCommand.execute();
        }

        boolean pfFinished = pathfindCommand != null && pathfindCommand.isFinished();

        Pose2d pose = drivetrain.getState().Pose;
        var speeds = drivetrain.getState().Speeds;
        if (executeCycles <= 5 || executeCycles % 20 == 0 || pfFinished) {
            System.out.println("[AutoCollect] cycle=" + executeCycles
                    + " pose=(" + String.format("%.2f", pose.getX()) + "," + String.format("%.2f", pose.getY()) + ")"
                    + " speeds=(" + String.format("%.3f", speeds.vxMetersPerSecond) + ","
                    + String.format("%.3f", speeds.vyMetersPerSecond) + ")"
                    + " heading=" + String.format("%.1f", pose.getRotation().getDegrees())
                    + " pfFinished=" + pfFinished);
        }

        DogLog.log("AutoCollect/HasTarget", detector.hasTarget());
        DogLog.log("AutoCollect/TargetCount", detector.getTargetCount());
        DogLog.log("AutoCollect/PathfindFinished", pfFinished);
        DogLog.log("AutoCollect/RobotX", pose.getX());
        DogLog.log("AutoCollect/RobotY", pose.getY());
    }

    @Override
    public boolean isFinished() {
        if (pathfindCommand != null && pathfindCommand.isFinished()) {
            System.out.println("[AutoCollect] PathfindingCommand finished!");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[AutoCollect] Ended (interrupted=" + interrupted + ")");
        PPHolonomicDriveController.setRotationTargetOverride(null);

        if (pathfindCommand != null) {
            pathfindCommand.end(interrupted);
        }
    }

    /**
     * Dynamically computes the desired heading: face detected ball clusters,
     * or face toward the target point if no balls are detected.
     * Returns Optional.empty() to fall back to trajectory rotation targets.
     */
    private Optional<Rotation2d> getRotationTargetOverride() {
        Pose2d currentPose = drivetrain.getState().Pose;

        OptionalDouble ballYaw = detector.getClusterYawDegrees();
        if (ballYaw.isPresent()) {
            double heading = currentPose.getRotation().getRadians() + Math.toRadians(ballYaw.getAsDouble());
            DogLog.log("AutoCollect/TargetHeadingDeg", Math.toDegrees(heading));
            return Optional.of(new Rotation2d(heading));
        }

        Rotation2d heading = computeHeadingToTarget(currentPose);
        DogLog.log("AutoCollect/TargetHeadingDeg", heading.getDegrees());
        return Optional.of(heading);
    }

    private Rotation2d computeHeadingToTarget(Pose2d currentPose) {
        Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        if (toTarget.getNorm() < 0.01) {
            return currentPose.getRotation();
        }
        return toTarget.getAngle();
    }
}
