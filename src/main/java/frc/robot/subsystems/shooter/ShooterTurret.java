package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

/**
 * Turret subsystem using shared NT table for bidirectional communication.
 */
public class ShooterTurret extends SubsystemBase {
    private static final double AT_TARGET_TOLERANCE_DEG = 2.0;

    private final int motorId;
    private TalonFX motor;
    private MotionMagicVoltage position = new MotionMagicVoltage(0).withEnableFOC(true);
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private double lastRequestedPositionRot = 0.0;

    private final Angle minRotation = Degrees.of(-90);
    private final Angle maxRotation = Degrees.of(90);

    /** Shared NT table */
    private final NetworkTable nt;
    private double prevNtTargetDeg = Double.NaN;

    public ShooterTurret() {
        nt = NetworkTableInstance.getDefault().getTable("Shooter").getSubTable("Turret");

        // Publish defaults
        nt.getEntry("TargetDeg").setDouble(0.0);
        nt.getEntry("Zero").setBoolean(false);
        nt.getEntry("TuneLive").setBoolean(false);
        nt.getEntry("MotorID").setDouble(TurretConstants.MOTOR_ID);
        nt.getEntry("kP").setDouble(TurretConstants.kP);
        nt.getEntry("kI").setDouble(TurretConstants.kI);
        nt.getEntry("kD").setDouble(TurretConstants.kD);
        nt.getEntry("kV").setDouble(TurretConstants.kV);
        nt.getEntry("kS").setDouble(TurretConstants.kS);
        nt.getEntry("kA").setDouble(TurretConstants.kA);
        nt.getEntry("CruiseVelo").setDouble(TurretConstants.MAX_VELOCITY);
        nt.getEntry("Acceleration").setDouble(TurretConstants.MAX_ACCEL);

        motorId = (int) nt.getEntry("MotorID").getDouble(TurretConstants.MOTOR_ID);
        this.motor = new TalonFX(motorId);

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = this.maxRotation.in(Rotations);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = this.minRotation.in(Rotations);

        cfg.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;
        cfg.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MAX_VELOCITY;
        cfg.MotionMagic.MotionMagicAcceleration = TurretConstants.MAX_ACCEL;

        cfg.Slot0.kP = TurretConstants.kP;
        cfg.Slot0.kI = TurretConstants.kI;
        cfg.Slot0.kD = TurretConstants.kD;
        cfg.Slot0.kV = TurretConstants.kV;
        cfg.Slot0.kS = TurretConstants.kS;
        cfg.Slot0.kA = TurretConstants.kA;

        cfg.Voltage.PeakForwardVoltage = TurretConstants.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -TurretConstants.MAX_VOLTS;
        this.motor.getConfigurator().apply(cfg);

        var talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    public Command requestPosition(Angle pos) {
        return runOnce(() -> setPosition(pos));
    }

    public void setPosition(Angle pos) {
        lastRequestedPositionRot = pos.in(Rotations);
        this.motor.setControl(this.position.withPosition(pos));
    }

    public Command stop() {
        return runOnce(() -> this.motor.stopMotor());
    }

    public Command zero() {
        return runOnce(() -> this.motor.setPosition(0.0));
    }

    public double getAngleDegrees() {
        return this.motor.getPosition().getValue().in(Degrees);
    }

    /* sim */
    private static final double gearRatio = TurretConstants.GEAR_RATIO;
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.0818917055, gearRatio),
            gearbox);

    @Override
    public void periodic() {
        // ─── 1. 从 NT 读取命令 ───
        // Zero
        if (nt.getEntry("Zero").getBoolean(false)) {
            motor.setPosition(0.0);
            lastRequestedPositionRot = 0.0;
            nt.getEntry("Zero").setBoolean(false);
        }

        // Target position
        double ntDeg = nt.getEntry("TargetDeg").getDouble(Double.NaN);
        if (!Double.isNaN(ntDeg) && ntDeg != prevNtTargetDeg) {
            prevNtTargetDeg = ntDeg;
            setPosition(Degrees.of(ntDeg));
        }

        // Live tuning
        if (nt.getEntry("TuneLive").getBoolean(false)) {
            cfg.Slot0.kP = nt.getEntry("kP").getDouble(TurretConstants.kP);
            cfg.Slot0.kI = nt.getEntry("kI").getDouble(TurretConstants.kI);
            cfg.Slot0.kD = nt.getEntry("kD").getDouble(TurretConstants.kD);
            cfg.Slot0.kV = nt.getEntry("kV").getDouble(TurretConstants.kV);
            cfg.Slot0.kS = nt.getEntry("kS").getDouble(TurretConstants.kS);
            cfg.Slot0.kA = nt.getEntry("kA").getDouble(TurretConstants.kA);
            cfg.MotionMagic.MotionMagicCruiseVelocity = nt.getEntry("CruiseVelo").getDouble(TurretConstants.MAX_VELOCITY);
            cfg.MotionMagic.MotionMagicAcceleration = nt.getEntry("Acceleration").getDouble(TurretConstants.MAX_ACCEL);
            motor.getConfigurator().apply(cfg);
        }

        // ─── 2. 写入状态到同一个 NT 表 ───
        double curDeg = motor.getPosition().getValue().in(Degrees);
        double requestedDeg = lastRequestedPositionRot * 360.0;
        double errDeg = requestedDeg - curDeg;
        boolean atTarget = Math.abs(errDeg) < AT_TARGET_TOLERANCE_DEG;

        nt.getEntry("CurrentAngle").setDouble(curDeg);
        nt.getEntry("RequestedPosition").setDouble(requestedDeg);
        nt.getEntry("Error").setDouble(errDeg);
        nt.getEntry("AtTarget").setBoolean(atTarget);
        nt.getEntry("Enabled").setBoolean(true);

        // DogLog 文件日志
        DogLog.log("ShooterTurret/CurrentAngle", curDeg);
        DogLog.log("ShooterTurret/Error", errDeg);
        DogLog.log("ShooterTurret/AtTarget", atTarget);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = motor.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        nt.getEntry("MotorVoltage").setDouble(motorVoltage.in(Volts));

        sim.setInputVoltage(motorVoltage.in(Volts));
        sim.update(0.020);

        talonFXSim.setRawRotorPosition(sim.getAngularPosition().times(gearRatio));
        talonFXSim.setRotorVelocity(sim.getAngularVelocity().times(gearRatio));

        nt.getEntry("SimPosition").setDouble(sim.getAngularPosition().times(gearRatio).in(Degrees));
    }

    public Pose3d getComponentPose3d() {
        return new Pose3d(
                0.131341, -0.046547, 0.391529,
                new Rotation3d(0, 0, this.motor.getPosition().getValue().in(Radians)));
    }
}
