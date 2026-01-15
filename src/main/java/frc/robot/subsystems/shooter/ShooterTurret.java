package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.String;

public class ShooterTurret extends SubsystemBase {
    private TalonFX motor;
    private CANcoder cantCoder;
    private MotionMagicVoltage position = new MotionMagicVoltage(0).withSlot(0);
    private Slot0Configs slot0Configs;

    /**
     * Maximum rotation in radians
     */
    private final Angle maxRotation = Radians.of(2);

    private final DoubleSubscriber kP = DogLog.tunable("ShooterTurret/kP", 0.0, x -> {
        slot0Configs.kP = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("ShooterTurret/kI", 0.0, x -> {
        slot0Configs.kI = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("ShooterTurret/kD", 0.0, x -> {
        slot0Configs.kD = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("ShooterTurret/kV", 0.0, x -> {
        slot0Configs.kV = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kS = DogLog.tunable("ShooterTurret/kS", 0.0, x -> {
        slot0Configs.kS = x;
        motor.getConfigurator().apply(slot0Configs);
    });

    public ShooterTurret() {
        this.motor = new TalonFX(0);
        this.cantCoder = new CANcoder(0);
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = this.cantCoder.getDeviceID();
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = this.maxRotation.in(Rotations);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        this.motor.getConfigurator().apply(cfg);
    }

    public Command requestPosition(Angle pos) {
        if (pos.lt(Radians.of(0)) || pos.gt(this.maxRotation)) {
            DogLog.logFault(String.format("[ShooterTurret] Invalid position (%.2f) requested >:(", pos));
            return runOnce(() -> {
            });
        } else {
            return runOnce(() -> {
                this.motor.setControl(this.position.withPosition(pos));
            });
        }
    }

    public Command stop() {
        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }
}
