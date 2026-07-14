package frc.robot.subsystems.drive;

import java.util.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleState;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

public class EagleSwerveTelemetry {
    private static final int NT_PUBLISH_LIMIT = 25; // How many times to wait before publishing to networktables again

    private int telemetryCounter = 0;
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public EagleSwerveTelemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }

        SmartDashboard.putData("Field", field);
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisVelocities> driveVelocity = driveStateTable
            .getStructTopic("Velocity", ChassisVelocities.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
            .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleVelocity> driveModuleTargets = driveStateTable
            .getStructArrayTopic("ModuleTargets", SwerveModuleVelocity.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
            .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
            .publish();

    /* Robot pose for field positioning */
    private final Field2d field = new Field2d();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleVelocity = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
            m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
            m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
            m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
            m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
    };

    /**
     * Accept the swerve drive state and telemeterize it to SmartDashboard and
     * SignalLogger.
     */
    public void telemeterize(SwerveDriveState state) {
        if (++telemetryCounter < NT_PUBLISH_LIMIT) {
            return;
        }
        telemetryCounter = 0;

        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveVelocity.set(state.Velocity);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Velocity", ChassisVelocities.struct, state.Velocity);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        field.setRobotPose(state.Pose);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleVelocity[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleVelocity[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    /**
     * Updates the target pose from the superstructure for visualization.
     * @param targetPose An optional containing the target pose, or empty if no target
     */
    public void setSuperstructureTargetPose(Optional<Pose2d> targetPose) {
        if (targetPose.isPresent()) {
            field.getObject("SuperstructureTarget").setPose(targetPose.get());
        } else {
            field.getObject("SuperstructureTarget")
                    .setPose(new Pose2d(-1000.0, -1000.0, new org.wpilib.math.geometry.Rotation2d()));
        }
    }

    /**
     * Updates the autonomous trajectory on the Field2d for visualization.
     * @param pathPoses The poses of the trajectory to display on the field.
     */
    public void setAutonomousTrajectory(List<Pose2d> pathPoses) {
        if(pathPoses == null) {
            pathPoses = new ArrayList<>();
        }
        field.getObject("AutonomousPath").setPoses(pathPoses);
    }
}
