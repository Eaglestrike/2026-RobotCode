package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
    private TalonFX motor;

    public ShooterHood() {
        motor = new TalonFX(0);
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.SensorToMechanismRatio = 362.0 / 32.0;
        cfg.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        cfg.MotionMagic.MotionMagicAcceleration = 20.0;
        cfg.Slot0.kP = 100.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 5.0;
        cfg.Slot0.kV = 1.2;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
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
        DogLog.tunable("ShooterTurret/kS", cfg.Slot0.kS, x -> {
            System.out.println("Updating kS to " + x);
            cfg.Slot0.kS = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterTurret/kV", cfg.Slot0.kV, x -> {
            System.out.println("Updating kV to " + x);
            cfg.Slot0.kV = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
        DogLog.tunable("ShooterTurret/kA", cfg.Slot0.kA, x -> {
            System.out.println("Updating kA to " + x);
            cfg.Slot0.kA = x;
            motor.getConfigurator().apply(cfg.Slot0);
        });
    }

    private final MotionMagicVoltage request = new MotionMagicVoltage(0);

    public Command goToPos(double position) {
        return runOnce(() -> {
            motor.setControl(request.withPosition(position));
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(-1.36);
        });
    }

    public Pose3d getComponentPose3d(Pose3d turretPose) {
        return new Pose3d(
                turretPose.getX() + 0.106,
                turretPose.getY(),
                turretPose.getZ() + 0.0713,
                new Rotation3d(0, this.motor.getPosition().getValue().in(Radians),
                        turretPose.getRotation().getZ()));
    }
}