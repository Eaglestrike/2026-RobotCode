package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.Rollers.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        motor.optimizeBusUtilization();
    }

    public Command start() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.VOLTS));
        });
    }

    public Command eject() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(-IntakeConstants.Rollers.EJECT_VOLTS));
        });
    }

    public Command jiggle() {
        return run(() -> {
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.JIGGLE_VOLTS));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(0));
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setControl(voltageRequest.withOutput(joystickPosition.getAsDouble() * IntakeConstants.Rollers.VOLTS));
        });
    }
}
