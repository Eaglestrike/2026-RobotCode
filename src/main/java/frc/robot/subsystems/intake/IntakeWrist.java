package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    public IntakeWrist() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(2 * Math.PI / IntakeConstants.Wrist.GEAR_RATIO);
        config.closedLoop
                .iZone(0.2)
                .iMaxAccum(1)
                .pid(IntakeConstants.Wrist.kP, IntakeConstants.Wrist.kI, IntakeConstants.Wrist.kD).feedForward
                .kS(IntakeConstants.Wrist.kS)
                .kV(IntakeConstants.Wrist.kV)
                .kA(IntakeConstants.Wrist.kA)
                // .kCosRatio(IntakeConstants.Wrist.GEAR_RATIO)
                .kCos(IntakeConstants.Wrist.kG);
        config.closedLoop.maxMotion
                .cruiseVelocity(IntakeConstants.Wrist.MAX_VEL_RAD_PER_S)
                .maxAcceleration(IntakeConstants.Wrist.MAX_ACC_RAD_PER_S2);

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(IntakeConstants.Wrist.STALL_LIMIT);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DogLog.tunable("IntakeWrist/kP", IntakeConstants.Wrist.kP, newkP -> {
            config.closedLoop.p(newkP);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
        DogLog.tunable("IntakeWrist/kI", IntakeConstants.Wrist.kI, newkI -> {
            config.closedLoop.i(newkI);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
        DogLog.tunable("IntakeWrist/kD", IntakeConstants.Wrist.kD, newkD -> {
            config.closedLoop.d(newkD);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });

        DogLog.tunable("IntakeWrist/kS", IntakeConstants.Wrist.kS, newkS -> {
            config.closedLoop.feedForward.kS(newkS);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
        DogLog.tunable("IntakeWrist/kV", IntakeConstants.Wrist.kV, newkV -> {
            config.closedLoop.feedForward.kV(newkV);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
        DogLog.tunable("IntakeWrist/kA", IntakeConstants.Wrist.kA, newkA -> {
            config.closedLoop.feedForward.kA(newkA);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
        DogLog.tunable("IntakeWrist/kG", IntakeConstants.Wrist.kG, newkG -> {
            config.closedLoop.feedForward.kG(newkG);
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });

        DogLog.tunable("IntakeWristRequest", 0.0, x -> {
            moveTo(Radians.of(x)).schedule();
        });
    }

    public Command moveTo(Angle position) {
        return runOnce(() -> {
            DogLog.log("IntakeWrist/RequestedPosition", position.in(Radians), "Radians");
            controller.setSetpoint(position.in(Radians), ControlType.kMAXMotionPositionControl);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.getEncoder().setPosition(IntakeConstants.Wrist.ENCODER_OFFSET_RAD);
        });
    }

    public Command kill() {
        return runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    public Command setManualVoltage(double joystickPosition) {
        return runOnce(() -> {
            motor.setVoltage(joystickPosition * IntakeConstants.Wrist.MAX_VOLTS);
        });
    }

    /* sim */
    private static final double gearRatio = 100.0;
    private final DCMotor gearbox = DCMotor.getNEO(1);
    // 90 deg to -36 deg
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                    gearbox,
                    0.0004086057,
                    gearRatio),
            gearbox,
            gearRatio,
            0.378839,
            -2.2,
            0,
            true,
            -2.2);

    private final SparkMaxSim sparkSim = new SparkMaxSim(motor, gearbox);

    @Override
    public void simulationPeriodic() {
        sparkSim.setBusVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = sparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
        DogLog.log("IntakeWrist/MotorVoltage", motorVoltage, "Volts");

        sim.setInputVoltage(motorVoltage);
        sim.update(0.020);

        sparkSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(
                        sim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(),
                0.02);
        DogLog.log("IntakeWrist/SimPosition", sparkSim.getPosition() * gearRatio, "Radians");
    }

    private final DoubleSubscriber intakeang = DogLog.tunable("intakeang", 0.0);

    public Pose3d getComponentPose3d() {
        /* Rotations.of(this.motor.getEncoder().getPosition()).in(Radians) */
        DogLog.log("IntakeWrist/AngleRad", Rotations.of(this.motor.getEncoder().getPosition()).in(Radians), "Radians");
        DogLog.log("IntakeWrist/AngleRot", this.motor.getEncoder().getPosition());
        return new Pose3d(
                -0.311259, 0.303973, 0.203525 + 0.039846,
                new Rotation3d(0, Rotations.of(this.motor.getEncoder().getPosition()).in(Radians), 0));
    }
}
