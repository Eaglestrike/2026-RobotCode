package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private TalonFX motor;
    private VoltageOut voltageOutReq = new VoltageOut(0);
    private final DoubleSubscriber voltage = DogLog.tunable("Feeder/Voltage", 4.5, "V");

    public Feeder() {
        motor = new TalonFX(11);
    }

    public Command startMotor() {
        return runOnce(() -> {
            motor.setControl(voltageOutReq.withOutput(voltage.get()));
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            motor.setControl(voltageOutReq.withOutput(0));
        });
    }
    

}
