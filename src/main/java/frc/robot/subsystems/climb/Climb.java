package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// These comments will give you a peek into my sanity as it goes down the drain.
public class Climb extends SubsystemBase {
    private final TalonFX m_Left = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private final TalonFX m_Right = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);


    public Climb() {
        // Left motor config
        MotorOutputConfigs LeftMotorConfigs = new MotorOutputConfigs();
        LeftMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        LeftMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // Right motor config
        MotorOutputConfigs RightMotorConfigs = new MotorOutputConfigs();
        RightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        RightMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // Create config object
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // I fear configuring these
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = ClimbConstants.kS;
        slot0.kG = ClimbConstants.kG;
        slot0.kV = ClimbConstants.kV;
        slot0.kA = ClimbConstants.kA;
        slot0.kP = ClimbConstants.kP;
        slot0.kI = ClimbConstants.kI;
        slot0.kD = ClimbConstants.kD;

        // Apply configurations
        m_Left.getConfigurator().apply(cfg);
        m_Left.getConfigurator().apply(LeftMotorConfigs);
        m_Right.getConfigurator().apply(cfg);
        m_Right.getConfigurator().apply(RightMotorConfigs);

    }

    // Move both motors to a specified position
    public Command moveTo(double position) {
        return runOnce(
                () -> {
                    m_Left.setControl(m_mmReq.withPosition(position).withSlot(0));
                    m_Right.setControl(m_mmReq.withPosition(position).withSlot(0));
                });
    }

    // Zero the encoders of both motors
    public Command zero() {
        return runOnce(
                () -> {
                    System.out.println("Zeroing Climb");
                    m_Left.setPosition(ClimbConstants.OFFSET);
                    m_Right.setPosition(ClimbConstants.OFFSET);
                });
    }

    // Disable both motors immediately
    public Command kill() {
        return runOnce(
                () -> {
                    m_Left.disable();
                    m_Right.disable();
                });
    }

    // I have no idea what this does lol, just copied from 2025 code and retrofitted
    // to fit this thing
    public Command setManualVoltage(double joystickPosition) {
        return run(
                () -> {
                    m_Left.setVoltage(
                            joystickPosition * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                    m_Right.setVoltage(
                            joystickPosition * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                });
    }
}
