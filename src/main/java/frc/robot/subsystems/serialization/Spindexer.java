package frc.robot.subsystems.serialization;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Spindexer extends SubsystemBase {
    private TalonFX motor;
    private VoltageOut voltageOutReq = new VoltageOut(0).withEnableFOC(true);

    public Spindexer() {
        motor = new TalonFX(SerializationConstants.SPINDEXER_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor.getConfigurator().apply(cfg);
        motor.optimizeBusUtilization();
    }

    public void startMotor() {
        motor.setControl(voltageOutReq.withOutput(SerializationConstants.SPINDEXER_NORMAL_VOLTS));
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setControl(
                    voltageOutReq.withOutput(joystickPosition.getAsDouble() * SerializationConstants.SPINDEXER_MAX_VOLTS));
        });
    }

    public void stopMotor() {
        motor.setControl(voltageOutReq.withOutput(0));
    }

    public void startKicking() {
        motor.setControl(voltageOutReq.withOutput(SerializationConstants.SPINDEXER_MAX_VOLTS));
    }
}
