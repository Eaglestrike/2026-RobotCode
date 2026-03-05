package frc.robot.subsystems.serialization;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
    public static final boolean USE_VELOCITY_CONTROL = false; // set to true to use velocity control instead of voltage control

    private TalonFX motor;
    private VoltageOut voltageOutReq = new VoltageOut(0).withEnableFOC(true);
    private VelocityTorqueCurrentFOC velocityTorqueCurrentReq = new VelocityTorqueCurrentFOC(0);
    private final DoubleSubscriber voltage = DogLog.tunable("Feeder/Voltage", SerializationConstants.FEEDER_MAX_VOLTS,
            "V");

    public Feeder() {
        motor = new TalonFX(SerializationConstants.FEEDER_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.Slot0.kS = SerializationConstants.kS;
        cfg.Slot0.kP = SerializationConstants.kP;
        cfg.Slot0.kI = SerializationConstants.kI;
        cfg.Slot0.kD = SerializationConstants.kD;
        cfg.Slot0.kV = SerializationConstants.kV;
        cfg.Slot0.kA = SerializationConstants.kA;
        cfg.CurrentLimits.SupplyCurrentLimit = SerializationConstants.CURRENT_LIMIT;
        motor.getConfigurator().apply(cfg);
        motor.optimizeBusUtilization();
        motor.stopMotor();
    }

    public void startMotor() {
        if (USE_VELOCITY_CONTROL) {
            motor.setControl(velocityTorqueCurrentReq.withVelocity(SerializationConstants.FEEDER_SPEED));
        }
        motor.setControl(voltageOutReq.withOutput(voltage.get()));
    }

    public void stopMotor() {
        motor.stopMotor();
    }

}
