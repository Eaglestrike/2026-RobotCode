package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.Utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.subsystems.shooter.sim.FuelSim;
import frc.robot.subsystems.shooter.solver.LUTSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver;
import frc.robot.subsystems.shooter.solver.ShooterSolver.ShotData;

public class Shooter extends SubsystemBase {
    private enum ShooterState {
        IDLE,
        AIMING,
        SHOOTING,
        DUCKING
    }

    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;
    private final EagleSwerveDrivetrain drive;

    private final FuelSim fuelSim = new FuelSim();

    private final ShooterSolver solver = new LUTSolver();

    private ShooterState currentState = ShooterState.AIMING;
    private ShooterState lastState = currentState;

    private double distanceToTarget = 0.0;

    public Shooter(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        turret = new ShooterTurret(drive);
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();

        if (RobotBase.isSimulation()) {
            fuelSim.registerRobot(
                    Inches.of(21.75).in(Meters),
                    Inches.of(21.75).in(Meters),
                    0.0254,
                    () -> drive.getState().Pose,
                    () -> drive.getState().Speeds);
            fuelSim.spawnStartingFuel();
            fuelSim.start();
            fuelSim.enableAirResistance();
            SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
                    .withName("Reset Fuel")
                    .ignoringDisable(true));
        }
    }

    public Command stowShooter() {
        return Commands.sequence(
                runOnce(() -> {
                    lastState = currentState;
                    currentState = ShooterState.IDLE;
                }),
                Commands.parallel(
                        turret.requestPosition(Rotations.of(ShooterConstants.TurretConstants.STOW_POSITION)),
                        hood.requestPosition(Degrees.of(ShooterConstants.HoodConstants.STOW_ANGLE)),
                        runOnce(() -> flywheels.requestVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY))));
    }

    public Command duck() {
        return Commands.sequence(
                runOnce(() -> {
                    lastState = currentState;
                    currentState = ShooterState.DUCKING;
                }),
                hood.requestPosition(Degrees.of(ShooterConstants.HoodConstants.STOW_ANGLE)),
                runOnce(() -> flywheels.requestVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY)));
    }

    public Command unDuck() {
        return runOnce(() -> currentState = lastState);
    }

    /**
     * Returns true if the robot is currently moving toward its own alliance zone.
     * Blue alliance: alliance zone is at lower X, so negative field vX means toward alliance.
     * Red alliance: alliance zone is at higher X, so positive field vX means toward alliance.
     */
    private boolean isMovingTowardAllianceZone() {
        Pose2d robotPose = drive.getState().Pose;
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getState().Speeds, robotPose.getRotation());
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                == DriverStation.Alliance.Blue;
        // Blue: toward alliance = negative vX; Red: toward alliance = positive vX
        return isBlue ? fieldSpeeds.vxMetersPerSecond < -0.25
                : fieldSpeeds.vxMetersPerSecond > 0.25;
    }

    public void setShooting(boolean shooting) {
        if (shooting) {
            lastState = currentState;
            currentState = ShooterState.SHOOTING;
        } else {
            // When shooting is turned off, remain in aiming mode so the hood/turret
            // will track the target but the flywheel won't spin up.
            lastState = currentState;
            currentState = ShooterState.AIMING;
        }
    }

    // called periodically by superstructure
    public void setFacing(Pose3d targetPose) {
        DogLog.forceNt.log("Shooter/CurrentState", currentState.toString());
        this.targetPose = targetPose;
        if (targetPose == null) {
            flywheels.requestVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            lastSolverResult = null;
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

        ChassisSpeeds robotSpeeds = drive.getState().Speeds;

        // Calculate tangential velocity of the offset (robot frame)
        double vTanX = -robotSpeeds.omegaRadiansPerSecond * ShooterConstants.TurretConstants.OFFSET.getY();
        double vTanY = robotSpeeds.omegaRadiansPerSecond * ShooterConstants.TurretConstants.OFFSET.getX();

        double shooterVx = robotSpeeds.vxMetersPerSecond + vTanX;
        double shooterVy = robotSpeeds.vyMetersPerSecond + vTanY;

        // 3. Convert the total shooter velocity to the Field Frame
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                new ChassisSpeeds(shooterVx, shooterVy, robotSpeeds.omegaRadiansPerSecond),
                robotPose.getRotation());
        
        DogLog.log("Shooter/TurretVel", fieldSpeeds);

        // Solve for shooter settings
        Pose3d shooterPose = new Pose3d(shooterPos, new Rotation3d());
        ShotData result = solver.calculateShot(shooterPose, fieldSpeeds,
                targetPose.getTranslation());

        lastSolverResult = result;
        if (!result.validShot()) {
            logValues();
            return;
        }

        DogLog.log("Shooter/PredictedTargetTranslation3d",
                new Translation3d[] { result.predictedTarget() });

        DogLog.log("Shooter/Solver/TargetFieldTurretAngle_deg", result.turretAngle().getDegrees());
        DogLog.log("Shooter/Solver/TargetHoodAngle_deg", result.hoodAngle().getDegrees());
        DogLog.log("Shooter/Solver/TargetFlywheelVel_rpm", result.rpm());

        // Convert field-relative turret angle to robot-relative
        double targetTurretAngleDeg = -(result.turretAngle().getDegrees() - robotPose.getRotation().getDegrees());
        targetTurretAngleDeg += frc.robot.subsystems.Superstructure.turretTrimDegrees;

        DogLog.log("Shooter/Solver/TargetTurretAngle_deg", targetTurretAngleDeg);

        // Command subsystems
        if (currentState != ShooterState.IDLE) {
            turret.setAngle(Degrees.of(targetTurretAngleDeg));
        }

        // If we're ducking, don't move the hood but spin flywheels at DUCK_RPM
        // when heading toward our alliance zone so we're ready to shoot on arrival.
        if (currentState == ShooterState.DUCKING) {
            if (isMovingTowardAllianceZone()) {
                flywheels.requestVelocity(ShooterConstants.FlywheelConstants.DUCK_RPM);
            } else {
                flywheels.requestVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            }
        } else if (currentState != ShooterState.IDLE) {
            double targetHoodAngleDeg = result.hoodAngle().getDegrees() + frc.robot.subsystems.Superstructure.hoodTrimDegrees;
            hood.setGoal(Degrees.of(targetHoodAngleDeg));
            if (currentState == ShooterState.SHOOTING) {
                flywheels.requestVelocity(result.rpm());
            } else {
                flywheels.requestVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY);
            }
        }

        if (RobotBase.isSimulation()) {
            if (Utils.getCurrentTimeSeconds() % 0.2 < 0.02) {
                // fuelSim.launchFuel(MetersPerSecond.of(
                // rpmToBallVel(requestRpm)), Degrees.of(requestHoodAngleDeg),
                // Degrees.of(-targetTurretAngleDeg),
                // Meters.of(ShooterConstants.TurretConstants.OFFSET_HEIGHT));
                fuelSim.launchFuel(MetersPerSecond.of(
                        rpmToBallVel(result.rpm())), Degrees.of(result.hoodAngle().getDegrees()),
                        Degrees.of(-targetTurretAngleDeg),
                        Meters.of(ShooterConstants.TurretConstants.OFFSET_HEIGHT));
                launchTime = Utils.getCurrentTimeSeconds();
            }
            if (FuelSim.Hub.BLUE_HUB.getScore() > prevScore) {
                prevScore = FuelSim.Hub.BLUE_HUB.getScore();
                double tof = Utils.getCurrentTimeSeconds() - launchTime;
                DogLog.log("Shooter/LastShot/TimeOfFlight", tof);
            }
        }
        logValues();
    }

    private double launchTime = 0.0;
    private double prevScore = 0.0;

    private double requestRpm = 0.0;
    private double requestHoodAngleDeg = 0.0;

    {
        DogLog.tunable("Shooter/RequestRPM", 0.0, rpm -> requestRpm = rpm);
        DogLog.tunable("Shooter/RequestHoodAngleDeg", 0.0, angle -> requestHoodAngleDeg = angle);
    }

    public void shootManual() {
        flywheels.requestVelocity(requestRpm);
        hood.setGoal(Degrees.of(requestHoodAngleDeg));
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
    private ShotData lastSolverResult = null;
    private Pose3d targetPose = null;

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
        DogLog.forceNt.log("Shooter/TurretOnTarget", turret.atTarget());
        DogLog.forceNt.log("Shooter/HoodOnTarget", hood.atTarget());
        DogLog.forceNt.log("Shooter/FlywheelReady", flywheels.atVelocity());
        DogLog.forceNt.log("Shooter/ReadyToShoot", isReady());

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
    }

    private static Transform3d toTransform(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    @Override
    public void simulationPeriodic() {
        fuelSim.updateSim();

    }
}
