package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShooterHood extends SubsystemBase {
    private TalonFX motor;
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

    public ShooterHood() {
        motor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = 362.0 / 32.0;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        cfg.MotionMagic.MotionMagicAcceleration = 1.0;
        cfg.Slot0.kP = 8.0;
        cfg.Slot0.kD = 0.2;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kV = 1.11;
        cfg.Slot0.kA = 0.04;
        cfg.Slot0.kG = 0.26;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        this.motor.getConfigurator().apply(cfg);

        DogLog.tunable("ShooterHood/CruiseVelo", cfg.MotionMagic.MotionMagicCruiseVelocity, x -> {
            System.out.println("Updating CruiseVelo to " + x);
            cfg.MotionMagic.MotionMagicCruiseVelocity = x;
            motor.getConfigurator().apply(cfg);
        });
        DogLog.tunable("ShooterHood/Acceleration", cfg.MotionMagic.MotionMagicAcceleration, x -> {
            System.out.println("Updating Acceleration to " + x);
            cfg.MotionMagic.MotionMagicAcceleration = x;
            motor.getConfigurator().apply(cfg);
        });

        DogLog.tunable("ShooterHood/kP", cfg.Slot0.kP, x -> {
            System.out.println("Updating kP to " + x);
            cfg.Slot0.kP = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterHood/kD", cfg.Slot0.kD, x -> {
            System.out.println("Updating kD to " + x);
            cfg.Slot0.kD = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterHood/kS", cfg.Slot0.kS, x -> {
            System.out.println("Updating kS to " + x);
            cfg.Slot0.kS = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterHood/kV", cfg.Slot0.kV, x -> {
            System.out.println("Updating kV to " + x);
            cfg.Slot0.kV = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterHood/kA", cfg.Slot0.kA, x -> {
            System.out.println("Updating kA to " + x);
            cfg.Slot0.kA = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterHood/kG", cfg.Slot0.kG, x -> {
            System.out.println("Updating kG to " + x);
            cfg.Slot0.kG = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });

        var talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        DogLog.tunable("HoodRequest", 0.0, x -> {
            goToPos(Radians.of(x)).schedule();
        });
    }

    public Command goToPos(Angle position) {
        return runOnce(() -> {
            motor.setControl(request.withPosition(position));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(Radians.of(-1.36));
        });
    }

    /* sim */
    private static final double gearRatio = 362.0 / 32.0;
    private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                    gearbox,
                    0.022,
                    gearRatio),
            gearbox,
            gearRatio,
            0.252559,
            -0.22,
            0,
            true,
            -0.22);

    @Override
    public void simulationPeriodic() {
        var talonFXSim = motor.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();
        DogLog.log("ShooterHood/MotorVoltage", motorVoltage.in(Volts), "Volts");

        sim.setInputVoltage(motorVoltage.in(Volts));
        sim.update(0.020);

        talonFXSim.setRawRotorPosition(sim.getAngleRads() * gearRatio);
        talonFXSim.setRotorVelocity(sim.getVelocityRadPerSec() * gearRatio);
        DogLog.log("ShooterHood/SimPosition", sim.getAngleRads(), "Radians");
    }

    public Pose3d getComponentPose3d(Pose3d turretPose) {
        // offset of the hood relative to the turret frame (forward, lateral, up)
        double offsetX = 0.106;
        double offsetY = 0.0;

        var yaw = turretPose.getRotation().getZ();

        // rotate the XY offset by the turret yaw to transform into field coordinates
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double fieldOffsetX = cos * offsetX - sin * offsetY;
        double fieldOffsetY = sin * offsetX + cos * offsetY;

        return new Pose3d(
                turretPose.getX() + fieldOffsetX,
                turretPose.getY() + fieldOffsetY,
                turretPose.getZ() + 0.0713,
                new Rotation3d(0, this.motor.getPosition().getValue().in(Radians), yaw));
    }
}