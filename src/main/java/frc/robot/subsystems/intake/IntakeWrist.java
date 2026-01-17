package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private TalonFX motor;
    private Slot0Configs slot0Configs = new Slot0Configs();

    public IntakeWrist() {
        motor = new TalonFX(0);
        var talonFXConfigs = new TalonFXConfiguration();

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 160;
        motionMagicConfigs.MotionMagicJerk = 1600;

        motor.getConfigurator().apply(talonFXConfigs);
    }

    private final DoubleSubscriber kS = DogLog.tunable("IntakeWrist/kS", 0.0, newKS -> {
        slot0Configs.kS = newKS;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("IntakeWrist/kV", 0.0, newKV -> {
        slot0Configs.kV = newKV;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kA = DogLog.tunable("IntakeWrist/kA", 0.0, newKA -> {
        slot0Configs.kA = newKA;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kP = DogLog.tunable("IntakeWrist/kP", 0.0, newKP -> {
        slot0Configs.kP = newKP;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("IntakeWrist/kI", 0.0, newKI -> {
        slot0Configs.kI = newKI;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("IntakeWrist/kD", 0.0, newKD -> {
        slot0Configs.kD = newKD;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final MotionMagicVoltage request = new MotionMagicVoltage(0);

    public Command goToPos(double position) {
        return runOnce(() -> {
            motor.setControl(request.withPosition(position));
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(0);
        });
    }

}