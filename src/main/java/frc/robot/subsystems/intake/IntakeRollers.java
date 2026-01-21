package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {

    private TalonFX motor;
    private Command startCommand;

    public IntakeRollers() {
        startCommand = run(() -> {
            motor.setVoltage(voltage.get());
        });

        motor = new TalonFX(0); // placeholder??

    }

    public void start() {
        startCommand.schedule();
    }

    public void stop() {
        startCommand.cancel();
    }

    private final DoubleSubscriber voltage = DogLog.tunable("IntakeRollers/Voltage", 1.0);

}
