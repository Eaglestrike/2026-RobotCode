package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// These comments will give you a peek into my sanity as it goes down the drain.
public class Climb extends SubsystemBase {
    private final TalonFX left = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private final TalonFX right = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
    private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    private final TalonFXConfiguration cfg = new TalonFXConfiguration();
    private final Slot0Configs slot0Configs = cfg.Slot0;
    private final DoubleSubscriber kS = DogLog.tunable("Climb/kS", 0.0, newKS -> {
        slot0Configs.kS = newKS;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kG = DogLog.tunable("Climb/kG", 0.0, newKG -> {
        slot0Configs.kG = newKG;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("Climb/kV", 0.0, newKV -> {
        slot0Configs.kV = newKV;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kA = DogLog.tunable("Climb/kA", 0.0, newKA -> {
        slot0Configs.kA = newKA;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kP = DogLog.tunable("Climb/kP", 0.0, newKP -> {
        slot0Configs.kP = newKP;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("Climb/kI", 0.0, newKI -> {
        slot0Configs.kI = newKI;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("Climb/kD", 0.0, newKD -> {
        slot0Configs.kD = newKD;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });

    public Climb() {
        // left motor config
        MotorOutputConfigs leftMotorConfigs = new MotorOutputConfigs();
        leftMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // right motor config
        MotorOutputConfigs rightMotorConfigs = new MotorOutputConfigs();
        rightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        slot0Configs.kS = ClimbConstants.kS;
        slot0Configs.kG = ClimbConstants.kG;
        slot0Configs.kV = ClimbConstants.kV;
        slot0Configs.kA = ClimbConstants.kA;
        slot0Configs.kP = ClimbConstants.kP;
        slot0Configs.kI = ClimbConstants.kI;
        slot0Configs.kD = ClimbConstants.kD;

        VoltageConfigs voltages = new VoltageConfigs();
        voltages.PeakForwardVoltage = ClimbConstants.MAX_VOLTS;
        voltages.PeakReverseVoltage = -ClimbConstants.MAX_VOLTS;

        // Apply configurations
        left.getConfigurator().apply(cfg);
        left.getConfigurator().apply(leftMotorConfigs);
        left.getConfigurator().apply(voltages);
        right.getConfigurator().apply(cfg);
        right.getConfigurator().apply(rightMotorConfigs);
        left.getConfigurator().apply(voltages);

        right.setControl(new Follower(ClimbConstants.LEFT_CLIMB_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    // Move both motors to a specified position
    public Command moveTo(double position) {
        return runOnce(
                () -> {
                    left.setControl(mmReq.withPosition(position).withSlot(0));
                    right.setControl(mmReq.withPosition(position).withSlot(0));
                });
    }

    // Zero the encoders of both motors
    public Command zero() {
        return runOnce(
                () -> {
                    System.out.println("Zeroing Climb");
                    left.setPosition(ClimbConstants.LEFT_OFFSET);
                    right.setPosition(ClimbConstants.RIGHT_OFFSET);
                });
    }

    // Disable both motors immediately
    public Command kill() {
        return runOnce(
                () -> {
                    left.disable();
                    right.disable();
                });
    }

    public Command setManualVoltage(double joystickPosition) {
        return run(
                () -> {
                    left.setVoltage(
                            joystickPosition * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                    right.setVoltage(
                            joystickPosition * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                });
    }
}
