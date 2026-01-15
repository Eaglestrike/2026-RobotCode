package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;
import java.lang.String;

public class ShooterTurret extends SubsystemBase {
    private TalonFX motor;
    private CANcoder cantCoder;
    private MotionMagicVoltage position = new MotionMagicVoltage(0).withSlot(0);
    private Slot0Configs slot0Configs;

    /**
     * Maximum rotation in radians
     */
    private final double maxRotation = 2d;

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
        this.motor.getConfigurator().apply(cfg);
    }

    /**
     * Request that the turret move to a specified position (duh)
     * 
     * @param pos - Position in radians
     */
    public Command requestPosition(float pos) {
        if (pos < 0 || pos > this.maxRotation) {
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
