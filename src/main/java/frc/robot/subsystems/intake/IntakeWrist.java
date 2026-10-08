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

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

public class IntakeWrist extends SubsystemBase {
    private final TalonFX motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final PositionVoltage positionReq = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut voltReq = new VoltageOut(0).withEnableFOC(true);

    enum WristState {
        MOVING, AT_TARGET, MANUAL, ZEROING, IDLE
    }

    private WristState currState = WristState.IDLE;
    private Angle targPos = IntakeConstants.Wrist.STOW_ANGLE;
    private double manualVolts = 0;

    private boolean zeroed = false;

    public IntakeWrist() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
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
            targPos = position;
            currState = WristState.MOVING;
        });
    }

    public Command zero() {
        return runOnce(() -> {
            currState = WristState.ZEROING;
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            manualVolts = joystickPosition.getAsDouble() * IntakeConstants.Wrist.MAX_VOLTS;
            currState = WristState.MANUAL;
        });
    }

    public boolean atPosition() {
        return Math.abs(motor.getPosition().getValueAsDouble() - targPos.in(Rotations)) < 0.01;
    }

    @Override
    public void periodic() {
        switch (currState) {
            case MOVING:
                motor.setControl(positionReq.withPosition(targPos));
                if (atPosition()) {
                    currState = WristState.AT_TARGET;
                }
                break;
            case AT_TARGET:
                motor.setControl(positionReq.withPosition(targPos));
                break;
            case MANUAL:
                motor.setControl(voltReq.withOutput(manualVolts));
                break;
            case ZEROING:
                motor.setPosition(IntakeConstants.Wrist.ENCODER_OFFSET);
                zeroed = true;
                DogLog.forceNt.log("IntakeWrist/IntakeWristZeroed", zeroed);
                currState = WristState.IDLE;
                break;
            case IDLE:
                motor.setControl(voltReq.withOutput(0));
                break;
        }
    }
}
