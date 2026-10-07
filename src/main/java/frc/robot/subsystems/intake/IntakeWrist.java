package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class IntakeWrist extends SubsystemBase {
    private final TalonFX motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final PositionVoltage positionReq = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut voltReq = new VoltageOut(0).withEnableFOC(true);

    private boolean zeroed = false;

    public IntakeWrist() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // todo: check this inversion
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Voltage.PeakForwardVoltage = IntakeConstants.Wrist.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -IntakeConstants.Wrist.MAX_VOLTS;
        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.Wrist.GEAR_RATIO;

        motor.getConfigurator().apply(cfg);
        motor.setControl(voltReq);

        DogLog.forceNt.log("IntakeWrist/IntakeWristZeroed", zeroed);
    }

    public Command requestPosition(Angle position) {
        return runOnce(() -> {
            motor.setControl(positionReq.withPosition(position));
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(IntakeConstants.Wrist.ENCODER_OFFSET);
            zeroed = true;

            DogLog.forceNt.log("IntakeWrist/IntakeWristZeroed", zeroed);
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            double manualVolts = joystickPosition.getAsDouble() * IntakeConstants.Wrist.MAX_VOLTS;
            motor.setVoltage(manualVolts);
        });
    }
}
