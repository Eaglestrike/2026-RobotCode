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
    private final DoubleSubscriber voltage = DogLog.tunable("IntakeRollers/Voltage", 1.0);

    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.Rollers.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    }

    public Command start() {
        return runOnce(() -> {
            double volts = voltage.get();
            if(volts > IntakeConstants.Rollers.MAX_VOLTS) {
                volts = IntakeConstants.Rollers.MAX_VOLTS;
            } else if (volts < -IntakeConstants.Rollers.MAX_VOLTS) {
                volts = -IntakeConstants.Rollers.MAX_VOLTS;
            }
            motor.setControl(voltageRequest.withOutput(volts));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(0));
        });
    }
}
