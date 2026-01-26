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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class ShooterTurret extends SubsystemBase {
    private TalonFX motor;
    private MotionMagicVoltage position = new MotionMagicVoltage(0).withSlot(0);
    private EagleSwerveDrivetrain drive;

    private final Angle minRotation = Degrees.of(-180);
    private final Angle maxRotation = Degrees.of(180);

    public ShooterTurret(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        this.motor = new TalonFX(0);
        var cfg = new TalonFXConfiguration();
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = this.maxRotation.in(Rotations);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = this.minRotation.in(Rotations);
        cfg.Feedback.SensorToMechanismRatio = 135.0 / 15.0;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        cfg.MotionMagic.MotionMagicAcceleration = 20.0;
        cfg.Slot0.kP = 100.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 5.0;
        cfg.Slot0.kV = 1.2;
        cfg.Slot0.kS = 0.0;
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
        DogLog.tunable("ShooterTurret/kI", cfg.Slot0.kI, x -> {
            System.out.println("Updating kI to " + x);
            cfg.Slot0.kI = x;
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

        DogLog.tunable("TurretRequest", 0.0, x -> {
            requestPosition(Radians.of(x)).schedule();
        });
    }

    public Command requestPosition(Angle pos) {
        return runOnce(() -> {
            DogLog.log("ShooterTurret/RequestedPosition", pos.in(Degrees), "Degrees");
            this.motor.setControl(this.position.withPosition(pos));
        });
    }

    public void setFacing(Pose2d to) {
        var pose = drive.getState().Pose.plus(new Transform2d(ShooterConstants.OFFSET, new Rotation2d()));
        var angle = to.getTranslation().minus(pose.getTranslation()).getAngle()
                .minus(pose.getRotation());

        this.motor.setControl(this.position.withPosition(angle.getRotations()));
    }

    public Command stop() {
        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }

    public Command zero() {
        return runOnce(() -> {
            this.motor.setPosition(0.0);
        });
    }

    /* sim */
    private static final double gearRatio = 135.0 / 15.0;
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
    }

    public Pose3d getComponentPose3d() {
        return new Pose3d(
                0.131341,
                -0.046547,
                0.391529,
                new Rotation3d(0, 0, this.motor.getPosition().getValue().in(Radians)));
    }
}
