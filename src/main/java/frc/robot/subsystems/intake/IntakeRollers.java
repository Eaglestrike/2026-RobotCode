package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeRollers extends SubsystemBase {
    private TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DoubleSubscriber voltage = DogLog.tunable("IntakeRollers/Voltage", IntakeConstants.Rollers.VOLTS);

    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.Rollers.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    }

    public Command start() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(voltage.get()));
        });
    }

    public Command eject() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(-voltage.get()));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(0));
        });
    }

    public Command setManualVoltage(double joystickPosition) {
        return run(() -> {
            System.out.println("Rollers Joystick pos: " + joystickPosition);
            motor.setControl(voltageRequest.withOutput(joystickPosition * IntakeConstants.Rollers.VOLTS));
        });
    }
}
