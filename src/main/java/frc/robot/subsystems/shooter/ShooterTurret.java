package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;

public class ShooterTurret extends SubsystemBase {
    private TalonFX motor;
    private CANcoder encoder;
    private PositionVoltage position = new PositionVoltage(0);
    private EagleSwerveDrivetrain drive;
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private CANcoderConfiguration encoderCfg = new CANcoderConfiguration();

    private boolean isRecentering = false;

    public ShooterTurret(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        this.motor = new TalonFX(TurretConstants.MOTOR_ID);
        this.encoder = new CANcoder(TurretConstants.CANCODER_ID);

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.FORWARD_LIMIT.in(Rotations);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.BACKWARD_LIMIT.in(Rotations);

        cfg.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM;
        cfg.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_SENSOR;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = TurretConstants.CANCODER_ID;

        encoderCfg.MagnetSensor.MagnetOffset = TurretConstants.MAGNET_OFFSET;
        encoderCfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

        cfg.Slot0.kP = TurretConstants.kP;
        cfg.Slot0.kI = TurretConstants.kI;
        cfg.Slot0.kD = TurretConstants.kD;
        cfg.Slot0.kV = TurretConstants.kV;
        cfg.Slot0.kS = TurretConstants.kS;
        cfg.Slot0.kA = TurretConstants.kA;

        cfg.Voltage.PeakForwardVoltage = TurretConstants.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -TurretConstants.MAX_VOLTS;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.motor.getConfigurator().apply(cfg);
        this.encoder.getConfigurator().apply(encoderCfg);
        motor.getPosition().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(10);
        motor.optimizeBusUtilization();
        var talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        DogLog.tunable("ShooterTurret/TurretRequest Deg", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Degrees.of(x)));
        });
    }

    /**
     * Calculate the nearest total setpoint
     * 
     * @param fromTotal Current angle ∈ [BACKWARD_LIMIT, FORWARD_LIMIT]
     * @param toOffset  Target angle ∈ [0, 360)
     */
    private Pair<Angle, Boolean> nearestWind(Angle fromTotal, Angle toOffset) {
        Angle fromOffset = Degrees.of(fromTotal.in(Degrees) % 360);

        Angle delta = toOffset.minus(fromOffset);
        Angle res = fromTotal.plus(delta);
        boolean wraps = false;

        while (res.gt(TurretConstants.FORWARD_LIMIT)) {
            res = res.minus(Degrees.of(360));
            wraps = true;
        }
        while (res.lt(TurretConstants.BACKWARD_LIMIT)) {
            res = res.plus(Degrees.of(360));
            wraps = true;
        }

        if (res.gt(TurretConstants.FORWARD_LIMIT)) {
            res = TurretConstants.FORWARD_LIMIT;
        } else if (res.lt(TurretConstants.BACKWARD_LIMIT)) {
            res = TurretConstants.BACKWARD_LIMIT;
        }

        return new Pair<Angle, Boolean>(res, wraps);
    }

    public double getAngleRad() {
        return (motor.getPosition().getValue().minus(TurretConstants.ENCODER_OFFSET).in(Rotations)) * Math.PI * 2;
    }

    /**
     * Sets control for the turret to move to requested position
     * Angle is measured from intake = 0 rotations, with clockwise positive
     *
     * @param pos Angle between -1 and 1 rotations
     * @return runOnce command (does not wait for movement to terminate; use
     *         atTarget() for that)
     */
    public Command requestPosition(Angle pos) {
        return runOnce(() -> {
            setAngle(pos);
        });
    }

    public void setAngle(Angle pos) {
        pos = pos.plus(TurretConstants.ENCODER_OFFSET);
        // DogLog.log("ShooterTurret/RequestedPosition", pos.in(Degrees), "Degrees");
        var wind = nearestWind(motor.getPosition().getValue(), pos);
        Angle setpoint = wind.getFirst();
        boolean wraps = wind.getSecond();

        if (wraps) {
            isRecentering = true;
        }

        // technically this should be in periodic() but setAngle()'s 5ms call interval
        // is faster >:(
        if (isRecentering && motor.getPosition().getValue().isNear(setpoint, TurretConstants.ANGLE_TOLERANCE)) {
            isRecentering = false;
        }

        double omega = RadiansPerSecond.of(drive.getState().Speeds.omegaRadiansPerSecond).in(RotationsPerSecond);
        this.motor.setControl(this.position.withPosition(setpoint).withVelocity(omega));
    }

    public Command stop() {
        return runOnce(() -> {
            motor.stopMotor();
        });
    }

    public void setManualInput(double joystickPosition) {

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        motor.getConfigurator().apply(cfg);

        motor.setVoltage(joystickPosition * TurretConstants.MAX_VOLTS);

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motor.getConfigurator().apply(cfg);

    }

    public boolean atTarget() {
        return !isRecentering;
    }

    @Override
    public void periodic() {
        DogLog.log("ShooterTurret/Position", motor.getPosition().getValue().minus(TurretConstants.ENCODER_OFFSET));
        // DogLog.log("ShooterTurret/TargetPosition", targetAngle.in(Degrees),
        // "Degrees");
        // DogLog.log("ShooterTurret/AtTarget", atTarget());
    }

    /* sim */
    private static final double gearRatio = TurretConstants.GEAR_RATIO;
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    gearbox,
                    0.0818917055,
                    gearRatio),
            gearbox);

    @Override
    public void simulationPeriodic() {
        var talonFXSim = motor.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();
        DogLog.log("ShooterTurret/MotorVoltage", motorVoltage.in(Volts), "Volts");

        sim.setInputVoltage(motorVoltage.in(Volts));
        sim.update(0.020);

        talonFXSim.setRawRotorPosition(sim.getAngularPosition().times(gearRatio));
        talonFXSim.setRotorVelocity(sim.getAngularVelocity().times(gearRatio));
        DogLog.log("ShooterTurret/SimPosition",
                sim.getAngularPosition().times(gearRatio).in(Degrees), "Degrees");
    }

    public Pose3d getComponentPose3d() {
        return new Pose3d(
                ShooterConstants.TurretConstants.OFFSET.getX(),
                ShooterConstants.TurretConstants.OFFSET.getY(),
                ShooterConstants.TurretConstants.OFFSET_HEIGHT,
                new Rotation3d(0, 0, -motor.getPosition().getValue().in(Radians)));
    }

}