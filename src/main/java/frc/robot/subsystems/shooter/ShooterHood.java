package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
    private TalonFX motor;
    private Slot0Configs slot0Configs;

    public ShooterHood() {
        motor = new TalonFX(0);
        var talonFXConfigs = new TalonFXConfiguration();

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        motor.getConfigurator().apply(talonFXConfigs);
    }

    private final DoubleSubscriber kS = DogLog.tunable("ShooterHood/kS", 0.0, newKS -> {
        slot0Configs.kS = newKS;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("ShooterHood/kV", 0.0, newKV -> {
        slot0Configs.kV = newKV;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kA = DogLog.tunable("ShooterHood/kA", 0.0, newKA -> {
        slot0Configs.kA = newKA;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kP = DogLog.tunable("ShooterHood/kP", 0.0, newKP -> {
        slot0Configs.kP = newKP;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("ShooterHood/kI", 0.0, newKI -> {
        slot0Configs.kI = newKI;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("ShooterHood/kD", 0.0, newKD -> {
        slot0Configs.kD = newKD;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public Command goToPos(double position) {
        return runOnce(() -> {
            motor.setControl(m_request.withPosition(position));
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(0);
        });
    }

}