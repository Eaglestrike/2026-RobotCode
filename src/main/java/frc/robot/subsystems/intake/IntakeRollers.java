package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

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
            System.out.println("starting normal intake voltage");
            motor.setControl(voltageRequest.withOutput(voltage.get()));
        });
    }

    public Command startHopperOut() {
        return runOnce(() -> {
            System.out.println("starting hopper out");
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.HOPPER_OUT_VOLTS));
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

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            System.out.println("Rollers Joystick pos: " + joystickPosition.getAsDouble());
            motor.setControl(voltageRequest.withOutput(joystickPosition.getAsDouble() * IntakeConstants.Rollers.VOLTS));
        });
    }
}
