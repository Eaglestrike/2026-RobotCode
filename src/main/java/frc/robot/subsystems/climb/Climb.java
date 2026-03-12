package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        return runToStall(ClimbConstants.DOWN_VOLTS, ClimbConstants.DOWN_STALL_CURRENT);
    }

    public Command moveToUp() {
        return runToStall(ClimbConstants.UP_VOLTS, ClimbConstants.UP_STALL_CURRENT);
    }

    private Command runToStall(double voltage, double stallCurrent) {
        return run(() -> {
            DogLog.log("Climb/VoltageOut", voltage);
            motor.setControl(voltReq.withOutput(voltage));
        })
                .until(() -> {
                    boolean motorStalled = motor.getSupplyCurrent().getValueAsDouble() > stallCurrent;
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
