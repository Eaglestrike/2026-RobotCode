package frc.robot.subsystems.climb;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

// These comments will give you a peek into my sanity as it goes down the drain.
public class Climb extends SubsystemBase {
    private final TalonFX left = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final TalonFX right = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    private final TalonFXConfiguration cfg = new TalonFXConfiguration();
    private final Slot0Configs slot0Configs = cfg.Slot0;
    private final DoubleSubscriber kS = DogLog.tunable("Climb/kS", ClimbConstants.kS, newKS -> {
        slot0Configs.kS = newKS;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kG = DogLog.tunable("Climb/kG", ClimbConstants.kG, newKG -> {
        slot0Configs.kG = newKG;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("Climb/kV", ClimbConstants.kV, newKV -> {
        slot0Configs.kV = newKV;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kA = DogLog.tunable("Climb/kA", ClimbConstants.kA, newKA -> {
        slot0Configs.kA = newKA;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kP = DogLog.tunable("Climb/kP", ClimbConstants.kP, newKP -> {
        slot0Configs.kP = newKP;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("Climb/kI", ClimbConstants.kI, newKI -> {
        slot0Configs.kI = newKI;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("Climb/kD", ClimbConstants.kD, newKD -> {
        slot0Configs.kD = newKD;
        left.getConfigurator().apply(slot0Configs);
        right.getConfigurator().apply(slot0Configs);
    });

    private ClimbConstants.ClimbState currentState = ClimbConstants.ClimbState.STOWED;


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
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        VoltageConfigs voltages = cfg.Voltage;
        voltages.PeakForwardVoltage = ClimbConstants.MAX_VOLTS;
        voltages.PeakReverseVoltage = -ClimbConstants.MAX_VOLTS;

        // Apply configurations
        left.getConfigurator().apply(cfg);
        left.getConfigurator().apply(leftMotorConfigs);
        right.getConfigurator().apply(cfg);
        right.getConfigurator().apply(rightMotorConfigs);
    }

    public Command moveTo(ClimbConstants.ClimbState state) {
        currentState = state;
        return new SelectCommand<ClimbConstants.ClimbState>(
                Map.ofEntries(
                        Map.entry(ClimbConstants.ClimbState.STOWED, stow()),
                        Map.entry(ClimbConstants.ClimbState.L1, climbL1()),
                        Map.entry(ClimbConstants.ClimbState.L2, climbL2()),
                        Map.entry(ClimbConstants.ClimbState.L3, climbL3())
                        ),
                () -> state);
    }

    private Command climbL1() {
        return Commands.sequence(
            moveTo(ClimbConstants.L1_EXTEND_HEIGHT, left),
            moveTo(ClimbConstants.CLIMBED_POSITION, left)
        );
    }

    private Command climbL2() {

        return Commands.sequence(
            climbL1(),
            moveTo(ClimbConstants.L2_EXTEND_HEIGHT, right),
            moveTo(ClimbConstants.CLIMBED_POSITION, right)
        );
    }

    private Command climbL3() {
        return Commands.sequence(
            climbL2(),
            moveTo(ClimbConstants.L3_EXTEND_HEIGHT, left),
            moveTo(ClimbConstants.CLIMBED_POSITION, left)
        );
    }

    private Command stow() {
        return Commands.sequence(
            moveTo(ClimbConstants.STOW_HEIGHT, left),
            moveTo(ClimbConstants.STOW_HEIGHT, right));
    }

    // Move one motor to a specified position
    private Command moveTo(double position, TalonFX motor) {
        return runOnce(
                () -> {
                    motor.setControl(mmReq.withPosition(position).withSlot(0));
                }).until(
                    () -> Math.abs(motor.getPosition().getValueAsDouble() - position) < 0.1);
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

    public Command setManualVoltage(DoubleSupplier leftjoystickPosition, DoubleSupplier rightjoystickPosition) {
        return run(
                () -> {
                    left.setVoltage(
                            leftjoystickPosition.getAsDouble() * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                    right.setVoltage(
                            rightjoystickPosition.getAsDouble() * ClimbConstants.MAX_VOLTS / ClimbConstants.MANUAL_RATIO);
                });
    }

    public ClimbConstants.ClimbState getState() {
        return currentState;
    }
}
