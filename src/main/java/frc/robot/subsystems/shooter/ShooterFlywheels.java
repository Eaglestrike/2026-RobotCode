package frc.robot.subsystems.shooter;

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
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class ShooterFlywheels extends SubsystemBase {
    private TalonFX motor;
    private MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = config.Slot0;

    private final DoubleSubscriber kS = DogLog.tunable("ShooterFlywheels/kS", FlywheelConstants.kS, newKS -> {
        slot0Configs.kS = newKS;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final DoubleSubscriber kV = DogLog.tunable("ShooterFlywheels/kV", FlywheelConstants.kV, newKV -> {
        slot0Configs.kV = newKV;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final DoubleSubscriber kA = DogLog.tunable("ShooterFlywheels/kA", FlywheelConstants.kA, newKA -> {
        slot0Configs.kA = newKA;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final DoubleSubscriber kP = DogLog.tunable("ShooterFlywheels/kP", FlywheelConstants.kP, newKP -> {
        slot0Configs.kP = newKP;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final DoubleSubscriber kI = DogLog.tunable("ShooterFlywheels/kI", FlywheelConstants.kI, newKI -> {
        slot0Configs.kI = newKI;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final DoubleSubscriber kD = DogLog.tunable("ShooterFlywheels/kD", FlywheelConstants.kD, newKD -> {
        slot0Configs.kD = newKD;
        motor.getConfigurator().apply(slot0Configs);
    });

    private final DoubleSubscriber targetAccel = DogLog.tunable("ShooterFlywheels/targetAccel",
            FlywheelConstants.TARGET_ACCEL, newTargetAccel -> {
                slot0Configs.kD = newTargetAccel;
                motor.getConfigurator().apply(slot0Configs);
            });

    public ShooterFlywheels() {
        VoltageConfigs voltages = config.Voltage;
        voltages.PeakForwardVoltage = FlywheelConstants.MAX_VOLTS;
        voltages.PeakReverseVoltage = -FlywheelConstants.MAX_VOLTS;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        slot0Configs.kS = FlywheelConstants.kS;
        slot0Configs.kV = FlywheelConstants.kV;
        slot0Configs.kA = FlywheelConstants.kA;
        slot0Configs.kP = FlywheelConstants.kP;
        slot0Configs.kI = FlywheelConstants.kI;
        slot0Configs.kD = FlywheelConstants.kD;

        motor = new TalonFX(FlywheelConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        motor.getConfigurator().apply(config);
    }

    public Command startAtVelocity(double targetVel) {
        return this.runOnce(() -> {
            motor.setControl(motionMagic.withVelocity(targetVel).withAcceleration(targetAccel.get()));
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            motor.stopMotor();
        });
    }
}