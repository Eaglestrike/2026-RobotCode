package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DoubleSubscriber voltage = DogLog.tunable("IntakeRollers/Voltage", 1.0);

    public IntakeRollers() {
        motor = new TalonFX(0);
    }

    public Command start() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(voltage.get()));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(0));
        });
    }
}
