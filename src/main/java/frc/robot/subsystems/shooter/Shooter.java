package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.solver.LUTSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver.ShotData;

public class Shooter extends SubsystemBase {
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;
    private final EagleSwerveDrivetrain drive;

    private final ShooterSolver solver = new LUTSolver();

    private boolean isShooting = false;
    private double distanceToTarget = 0.0;

    public Shooter(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        turret = new ShooterTurret(drive);
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();
    }

    public Command stowShooter() {
        return Commands.parallel(
                turret.requestPosition(Rotations.of(ShooterConstants.TurretConstants.STOW_POSITION)),
                hood.goToPos(Degrees.of(ShooterConstants.HoodConstants.STOW_ANGLE)),
                runOnce(() -> flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY)));
    }

    public void setShooting(boolean shooting) {
        isShooting = shooting;
    }

    // called periodically by superstructure
    public void setFacing(Pose3d targetPose) {
        this.targetPose = targetPose;
        if (targetPose == null) {
            flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            lastSolverResult = null;
            predictedTrajectoryPoses = new Pose3d[0];
            logValues();
            return;
        }
        Pose2d robotPose = drive.getState().Pose;

        // Compute shooter position in field frame
        Translation2d offsetField = ShooterConstants.TurretConstants.OFFSET
                .rotateBy(robotPose.getRotation());
        Translation2d shooterPos2d = robotPose.getTranslation().plus(offsetField);
        Translation3d shooterPos = new Translation3d(
                shooterPos2d.getX(), shooterPos2d.getY(),
                ShooterConstants.TurretConstants.OFFSET_HEIGHT);

        Translation3d deltaField = targetPose.getTranslation().minus(shooterPos);
        distanceToTarget = Math.sqrt(deltaField.getX() * deltaField.getX()
                + deltaField.getY() * deltaField.getY());

        // Compute field-relative robot velocity
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getState().Speeds,
                robotPose.getRotation());

        // Solve for shooter settings
        Pose3d shooterPose = new Pose3d(shooterPos, new Rotation3d());
        ShotData result = solver.calculateShot(shooterPose, fieldSpeeds,
                targetPose.getTranslation());

        lastSolverResult = result;
        if (!result.validShot()) {
            logValues();
            return;
        }

        DogLog.log("Shooter/Solver/TargetFieldTurretAngle_deg", result.turretAngle().getDegrees());
        DogLog.log("Shooter/Solver/TargetHoodAngle_deg", result.hoodAngle().getDegrees());
        DogLog.log("Shooter/Solver/TargetFlywheelVel_rpm", result.rpm());

        // Convert field-relative turret angle to robot-relative
        double targetTurretAngleDeg = -(result.turretAngle().getDegrees() - robotPose.getRotation().getDegrees());
        DogLog.log("Shooter/Solver/TargetTurretAngle_deg", targetTurretAngleDeg);

        // Command subsystems
        turret.setAngle(Degrees.of(result.hoodAngle().getDegrees()));
        hood.setGoal(Degrees.of(targetTurretAngleDeg));

        if (isShooting) {
            flywheels.setVelocity(result.rpm());
        } else {
            flywheels.setVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
        }

        buildTrajectoryVisualization(shooterPos, fieldSpeeds);
        logValues();
    }

    public boolean isReady() {
        return turret.atTarget() && hood.atTarget() && flywheels.atVelocity();
    }

    public ShooterFlywheels getFlywheels() {
        return flywheels;
    }

    public ShooterTurret getTurret() {
        return turret;
    }

    public ShooterHood getHood() {
        return hood;
    }

    // ================================================================
    // Trajectory visualization (for AdvantageScope 3D Field)
    // ================================================================
    private Pose3d[] predictedTrajectoryPoses = new Pose3d[0];
    private ShotData lastSolverResult = null;
    private Pose3d targetPose = null;

    private void buildTrajectoryVisualization(Translation3d shooterPos, ChassisSpeeds fieldSpeeds) {
        if (lastSolverResult == null || !lastSolverResult.validShot()) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        double launchAngleRad = Math.toRadians(lastSolverResult.hoodAngle().getDegrees());
        double turretAngleRad = lastSolverResult.turretAngle().getRadians();

        // Recover launch velocity from RPM → RPS → ball vel (m/s)
        double launchVelocity = rpmToBallVel(lastSolverResult.rpm());

        // Ball's field-frame initial velocity = launch velocity + robot velocity
        double ct = Math.cos(launchAngleRad);
        double vxF = launchVelocity * ct * Math.cos(turretAngleRad) + fieldSpeeds.vxMetersPerSecond;
        double vyF = launchVelocity * ct * Math.sin(turretAngleRad) + fieldSpeeds.vyMetersPerSecond;
        double vzF = launchVelocity * Math.sin(launchAngleRad);

        // Estimate time of flight from distance / horizontal speed
        Translation3d predictedTarget = lastSolverResult.predictedTarget();
        double dx = predictedTarget.getX() - shooterPos.getX();
        double dy = predictedTarget.getY() - shooterPos.getY();
        double horizDist = Math.sqrt(dx * dx + dy * dy);
        double horizSpeed = Math.sqrt(vxF * vxF + vyF * vyF);
        double tFlight = horizDist / Math.max(horizSpeed, 1e-6);
        if (!Double.isFinite(tFlight) || tFlight <= 0) {
            predictedTrajectoryPoses = new Pose3d[0];
            return;
        }

        // Sample ~20 points along the trajectory (gravity-only, consistent with
        // DesmosSolver)
        int n = 20;
        double dtVis = tFlight / n;
        predictedTrajectoryPoses = new Pose3d[n + 1];

        double px = shooterPos.getX(), py = shooterPos.getY(), pz = shooterPos.getZ();
        double vx = vxF, vy = vyF, vz = vzF;
        double g = 9.81;

        for (int i = 0; i <= n; i++) {
            predictedTrajectoryPoses[i] = new Pose3d(px, py, Math.max(pz, 0), new Rotation3d());
            if (i < n) {
                // Gravity-only integration (no drag — matches DesmosSolver model)
                vz -= g * dtVis;
                px += vx * dtVis;
                py += vy * dtVis;
                pz += vz * dtVis;
            }
        }
    }

    private double rpmToBallVel(double rpm) {
        // linear conv
        double wheelCircumference = Math.PI * 0.1016; // 4 inch diameter wheel
        return rpm * wheelCircumference / 60.0;
    }

    private void logValues() {
        DogLog.log("Shooter/HasTarget", targetPose != null);
        DogLog.log("Shooter/DistanceToTarget", distanceToTarget);
        DogLog.log("Shooter/RobotYaw_deg", drive.getState().Pose.getRotation().getDegrees());
        DogLog.log("Shooter/ActualHoodAngle_deg", hood.getAngle().in(Degrees));
        DogLog.log("Shooter/TurretOnTarget", turret.atTarget());
        DogLog.log("Shooter/HoodOnTarget", hood.atTarget());
        DogLog.log("Shooter/FlywheelReady", flywheels.atVelocity());
        DogLog.log("Shooter/ReadyToShoot", isReady());

        // Solver diagnostics
        DogLog.log("Shooter/Solver/Success", lastSolverResult != null && lastSolverResult.validShot());

        // 3D visualization poses (field-absolute, with odom)
        Pose2d robotPose = drive.getState().Pose;
        Pose3d robotPose3d = new Pose3d(
                robotPose.getX(), robotPose.getY(), 0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
        Pose3d turretLocal = turret.getComponentPose3d();
        Pose3d hoodLocal = hood.getComponentPose3d(turretLocal);
        DogLog.log("Shooter/TurretPose3d", new Pose3d[] { robotPose3d.transformBy(toTransform(turretLocal)) });
        DogLog.log("Shooter/HoodPose3d", new Pose3d[] { robotPose3d.transformBy(toTransform(hoodLocal)) });

        if (targetPose != null) {
            DogLog.log("Shooter/TargetPose3d",
                    new Pose3d[] { targetPose });
        }
        if (predictedTrajectoryPoses.length > 0) {
            DogLog.log("Shooter/PredictedTrajectory", predictedTrajectoryPoses);
        }
    }

    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }
}
