package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import static edu.wpi.first.units.Units.Volts;

public class Hopper extends SubsystemBase {
    private TalonFX motor;
    private VoltageOut voltageOutReq;
    private final DoubleSubscriber voltage = DogLog.tunable("Hopper/Voltage", 4.5, "V", newVoltage -> {
        motor.setVoltage(newVoltage);
    });

    public Hopper() {
        motor = new TalonFX(11);
    }

    public Command startMotor() {
        return runOnce(() -> {
            motor.setControl(voltageOutReq.withOutput(10)); // hardcoded voltage
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
            motor.setControl(voltageOutReq.withOutput(0));
        });
    }

    // @Override
    // public void periodic() {
    // }
}