package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
    private TalonFX motor;
    private VoltageOut voltageOutReq = new VoltageOut(0);
    private final DoubleSubscriber voltage = DogLog.tunable("Feeder/Voltage", SerializationConstants.FEEDER_MAX_VOLTS,
            "V");

    public Feeder() {
        motor = new TalonFX(SerializationConstants.FEEDER_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(cfg);
        motor.stopMotor();
    }

    public void startMotor() {
        motor.setControl(voltageOutReq.withOutput(voltage.get()));
    }

    public void stopMotor() {
        motor.setControl(voltageOutReq.withOutput(0));
    }

}
