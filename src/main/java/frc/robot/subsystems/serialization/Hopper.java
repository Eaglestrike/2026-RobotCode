package frc.robot.subsystems.serialization;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Hopper extends SubsystemBase {
    private TalonFX motor;
    private VoltageOut voltageOutReq = new VoltageOut(0);
    private final DoubleSubscriber voltage = DogLog.tunable("Hopper/Voltage",
            SerializationConstants.HOPPER_NORMAL_VOLTS,
            "V");

    public Hopper() {
        motor = new TalonFX(SerializationConstants.HOPPER_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    }

    public void startMotor() {
        motor.setControl(voltageOutReq.withOutput(voltage.get()));
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setControl(
                    voltageOutReq.withOutput(joystickPosition.getAsDouble() * SerializationConstants.HOPPER_MAX_VOLTS));
        });
    }

    public void stopMotor() {
        motor.setControl(voltageOutReq.withOutput(0));
    }

    public void startKicking() {
        motor.setControl(voltageOutReq.withOutput(SerializationConstants.HOPPER_MAX_VOLTS));
    }
}
