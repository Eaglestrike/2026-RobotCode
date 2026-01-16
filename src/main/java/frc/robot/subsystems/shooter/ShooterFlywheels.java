package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.*;

public class ShooterFlywheels extends SubsystemBase {
    private TalonFX motor;
    private MotionMagicVelocityVoltage motionMagic;
    private Slot0Configs slot0Configs = new Slot0Configs();

    private final DoubleSubscriber kS = DogLog.tunable("ShooterFlywheels/kS", 0.0, newKS -> {
        slot0Configs.kS = newKS;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("ShooterFlywheels/kV", 0.0, newKV -> {
        slot0Configs.kV = newKV;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kA = DogLog.tunable("ShooterFlywheels/kA", 0.0, newKA -> {
        slot0Configs.kA = newKA;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kP = DogLog.tunable("ShooterFlywheels/kP", 0.0, newKP -> {
        slot0Configs.kP = newKP;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("ShooterFlywheels/kI", 0.0, newKI -> {
        slot0Configs.kI = newKI;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("ShooterFlywheels/kD", 0.0, newKD -> {
        slot0Configs.kD = newKD;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber targetAccel = DogLog.tunable("ShooterFlywheels/targetAccel", 0.0);

    public ShooterFlywheels() {
        VoltageConfigs voltages = new VoltageConfigs();
        voltages.PeakForwardVoltage = TODO;
        voltages.PeakReverseVoltage = -TODO;
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Voltage = voltages;
        motor = new TalonFX(0, "rio");
        motor.getConfigurator().apply(config);
    }

    public Command startAtVelocity(double targetVel) {
        return this.runOnce(() -> {
            motor.setControl(new MotionMagicVelocityVoltage(targetVel).withAcceleration(targetAccel.get()));
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            motor.stopMotor();
        });
    }
}