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
    private final TalonFX master = new TalonFX(ClimbConstants.MASTER_CLIMB_MOTOR_ID, "rio");
    private final TalonFX follower = new TalonFX(ClimbConstants.FOLLOWER_CLIMB_MOTOR_ID, "rio");
    private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    private final TalonFXConfiguration cfg = new TalonFXConfiguration();
    private final Slot0Configs slot0Configs = cfg.Slot0;
    private final DoubleSubscriber kS = DogLog.tunable("ShooterFlywheels/kS", 0.0, newKS -> {
        slot0Configs.kS = newKS;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kG = DogLog.tunable("ShooterFlywheels/kG", 0.0, newKG -> {
        slot0Configs.kG = newKG;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("ShooterFlywheels/kV", 0.0, newKV -> {
        slot0Configs.kV = newKV;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kA = DogLog.tunable("ShooterFlywheels/kA", 0.0, newKA -> {
        slot0Configs.kA = newKA;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kP = DogLog.tunable("ShooterFlywheels/kP", 0.0, newKP -> {
        slot0Configs.kP = newKP;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("ShooterFlywheels/kI", 0.0, newKI -> {
        slot0Configs.kI = newKI;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("ShooterFlywheels/kD", 0.0, newKD -> {
        slot0Configs.kD = newKD;
        master.getConfigurator().apply(slot0Configs);
        follower.getConfigurator().apply(slot0Configs);
    });

    public Climb() {
        // master motor config
        MotorOutputConfigs masterMotorConfigs = new MotorOutputConfigs();
        masterMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        masterMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // follower motor config
        MotorOutputConfigs followerMotorConfigs = new MotorOutputConfigs();
        followerMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

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
        master.getConfigurator().apply(cfg);
        master.getConfigurator().apply(masterMotorConfigs);
        master.getConfigurator().apply(voltages);
        follower.getConfigurator().apply(cfg);
        follower.getConfigurator().apply(followerMotorConfigs);
        master.getConfigurator().apply(voltages);

        follower.setControl(new Follower(ClimbConstants.MASTER_CLIMB_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    // Move both motors to a specified position
    public Command moveTo(double position) {
        return runOnce(
                () -> {
                    master.setControl(mmReq.withPosition(position).withSlot(0));
                });
    }

    // Zero the encoders of both motors
    public Command zero() {
        return runOnce(
                () -> {
                    System.out.println("Zeroing Climb");
                    master.setPosition(ClimbConstants.OFFSET);
                    follower.setPosition(ClimbConstants.OFFSET);
                });
    }

    // Disable both motors immediately
    public Command kill() {
        return runOnce(
                () -> {
                    master.disable();
                    follower.disable();
                });
    }

    public Command setManualVoltage(double joystickPosition) {
        return run(
                () -> {
                    master.setVoltage(
                            joystickPosition * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                });
    }
}
