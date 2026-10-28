package frc.robot.subsystems.serialization;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Kicker extends SubsystemBase {
    private final TalonFX motor;
    private final VoltageOut voltageOutReq = new VoltageOut(0).withEnableFOC(true);

    public Kicker() {
        motor = new TalonFX(SerializationConstants.KICKER_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits.SupplyCurrentLimit = SerializationConstants.CURRENT_LIMIT;
        motor.getConfigurator().apply(cfg);
        motor.optimizeBusUtilization();
    }

    public void startMotor() {
        motor.setControl(voltageOutReq.withOutput(SerializationConstants.KICKER_MAX_VOLTS));
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setControl(
                    voltageOutReq
                            .withOutput(joystickPosition.getAsDouble() * SerializationConstants.SPINDEXER_MAX_VOLTS));
        });
    }

}
