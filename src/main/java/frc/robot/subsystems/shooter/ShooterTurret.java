package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterTurret extends SubsystemBase {
    private TalonFX motor;
    private PositionVoltage position = new PositionVoltage(0).withEnableFOC(true);
    private EagleSwerveDrivetrain drive;
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private Angle targetAngle = Degrees.of(0);

    private final Angle minRotation = Degrees.of(0);
    private final Angle maxRotation = Degrees.of(180);

    private final DigitalInput photoelectric;
    private static final boolean PHOTOELECTRIC_ACTIVE_LOW = true; // set to false if your sensor outputs HIGH when
                                                                  // broken

    public ShooterTurret(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        this.photoelectric = new DigitalInput(TurretConstants.PHOTOELECTRIC_DIO);
        this.motor = new TalonFX(TurretConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);

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
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.motor.getConfigurator().apply(cfg);

        DogLog.tunable("ShooterTurret/CruiseVelo", cfg.MotionMagic.MotionMagicCruiseVelocity, x -> {
            System.out.println("Updating CruiseVelo to " + x);
            cfg.MotionMagic.MotionMagicCruiseVelocity = x;
            motor.getConfigurator().apply(cfg);
        });
        DogLog.tunable("ShooterTurret/Acceleration", cfg.MotionMagic.MotionMagicAcceleration, x -> {
            System.out.println("Updating Acceleration to " + x);
            cfg.MotionMagic.MotionMagicAcceleration = x;
            motor.getConfigurator().apply(cfg);
        });

        DogLog.tunable("ShooterTurret/kP", cfg.Slot0.kP, x -> {
            System.out.println("Updating kP to " + x);
            cfg.Slot0.kP = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterTurret/kD", cfg.Slot0.kD, x -> {
            System.out.println("Updating kD to " + x);
            cfg.Slot0.kD = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterTurret/kV", cfg.Slot0.kV, x -> {
            System.out.println("Updating kV to " + x);
            cfg.Slot0.kV = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterTurret/kS", cfg.Slot0.kS, x -> {
            System.out.println("Updating kS to " + x);
            cfg.Slot0.kS = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });

        var talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        DogLog.tunable("ShooterTurret/TurretRequest Degrees", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Degrees.of(x)));
        });
    }

    public Command requestPosition(Angle pos) {
        return runOnce(() -> {
            setAngle(pos);
        });
    }

    /**
     * Directly set the turret target angle (for use by Shooter in periodic())
     */
    public void setAngle(Angle pos) {
        this.targetAngle = pos;
        this.motor.setControl(this.position.withPosition(pos));
    }

    public void setFacing(Pose2d to) {
        // TODO: Maybe account for robot's velocity when calculating angle to turn to
        var pose = drive.getState().Pose.plus(new Transform2d(TurretConstants.OFFSET, new Rotation2d()));
        var angle = to.getTranslation().minus(pose.getTranslation()).getAngle()
                .minus(pose.getRotation());
        // TODO: Check if this normalizes between -180 to 180 correctly
        targetAngle = Rotations.of(angle.getRotations());

        this.motor.setControl(this.position.withPosition(angle.getRotations()));
    }

    public Command stop() {
        return runOnce(() -> {
            motor.stopMotor();
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setVoltage(joystickPosition.getAsDouble() * TurretConstants.MAX_VOLTS);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(TurretConstants.ENCODER_OFFSET);
        });
    }

    public boolean atTarget() {
        return motor.getPosition().getValue().isNear(targetAngle, TurretConstants.ANGLE_TOLERANCE);
    }

    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    @Override
    public void periodic() {
        DogLog.log("ShooterTurret/Position", motor.getPosition().getValue().in(Degrees), "Degrees");
        DogLog.log("ShooterTurret/Photoelectric", photoelectric.get() ? 1 : 0, "Boolean");
        DogLog.log("ShooterTurret/AtTarget", atTarget() ? 1 : 0, "Boolean");
    }

    public boolean isZeroSensorGreen() {
        boolean raw = photoelectric.get();
        return PHOTOELECTRIC_ACTIVE_LOW ? !raw : raw;
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
        DogLog.log("ShooterTurret/SimPosition", sim.getAngularPosition().times(gearRatio).in(Degrees), "Degrees");

        DogLog.log("ShooterTurret/Photoelectric", photoelectric.get());

    }

    public Pose3d getComponentPose3d() {
        return new Pose3d(
                0.131341,
                -0.046547,
                0.391529,
                new Rotation3d(0, 0, this.motor.getPosition().getValue().in(Radians)));
    }

}