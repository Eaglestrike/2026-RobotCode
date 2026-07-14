package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    private final TalonFX motor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final VoltageOut voltReq = new VoltageOut(0).withEnableFOC(true);

    public Climb() {
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        VoltageConfigs voltages = cfg.Voltage;
        voltages.PeakForwardVoltage = ClimbConstants.MAX_VOLTS;
        voltages.PeakReverseVoltage = -ClimbConstants.MAX_VOLTS;

        motor.getConfigurator().apply(cfg);
        motor.getConfigurator().apply(motorConfigs);
    }

    public Command moveToStow() {
        return run(() -> {
            DogLog.log("Climb/VoltageOut", ClimbConstants.DOWN_VOLTS);
            motor.setControl(voltReq.withOutput(ClimbConstants.DOWN_VOLTS));
        })
                .until(() -> {
                    boolean motorStalled = motor.getSupplyCurrent()
                            .getValueAsDouble() > ClimbConstants.DOWN_STALL_CURRENT;
                    DogLog.log("Climb/MotorStalled", motorStalled);
                    return motorStalled;
                })
                .finallyDo(() -> {
                    DogLog.log("Climb/FinallyDone", true);
                    motor.setControl(voltReq.withOutput(0));
                });
    }

    public Command moveToUp() {
        return run(() -> {
            DogLog.log("Climb/VoltageOut", ClimbConstants.UP_VOLTS);
            motor.setControl(voltReq.withOutput(ClimbConstants.UP_VOLTS));
        })
                .until(() -> {
                    boolean motorStalled = motor.getSupplyCurrent()
                            .getValueAsDouble() > ClimbConstants.UP_STALL_CURRENT;
                    DogLog.log("Climb/MotorStalled", motorStalled);
                    return motorStalled;
                })
                .finallyDo(() -> {
                    DogLog.log("Climb/FinallyDone", true);
                    motor.setControl(voltReq.withOutput(0));
                });
    }

    public Command setManualVoltage(DoubleSupplier joystickPos) {
        return run(
                () -> {
                    motor.setVoltage(
                            joystickPos.getAsDouble() * ClimbConstants.MAX_VOLTS
                                    / ClimbConstants.MANUAL_RATIO);
                });
    }
}
