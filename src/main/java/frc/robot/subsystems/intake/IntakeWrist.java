package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;


public class IntakeWrist extends SubsystemBase {
    private enum State {
        IDLE, MOVING_TO_GOAL, MANUAL, JIGGLING
    }

    private final TalonFX motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final VoltageOut voltReq = new VoltageOut(0).withEnableFOC(true);

    private State state = State.IDLE;

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

        DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);
    }

    public Command requestPosition(Angle position) {
        double voltage = position.isNear(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE), 0.5) ? IntakeConstants.Wrist.UP_VOLTS : IntakeConstants.Wrist.DOWN_VOLTS;
        double stallCurrent = position.isNear(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE), 0.5) ? IntakeConstants.Wrist.UP_STALL_CURRENT : IntakeConstants.Wrist.DOWN_STALL_CURRENT;
        DogLog.log("Intake/WristVoltage", voltage);
        DogLog.log("Intake/StallCurrent", stallCurrent);
        return Commands.select(
                Map.ofEntries(
                        Map.entry(true,
                                Commands.sequence(
                                        runOnce(() -> {
                                            motor.setControl(voltReq.withOutput(voltage));
                                            state = State.MOVING_TO_GOAL;
                                        }),
                                        Commands.waitUntil(() -> motor.getSupplyCurrent().getValueAsDouble() > stallCurrent),
                                        runOnce(() -> {
                                            motor.setControl(voltReq.withOutput(0));
                                            motor.setPosition(position); // Cos stalling messes up the encoder
                                            state = State.IDLE;
                                        })
                                )),
                        Map.entry(false, Commands.print("Wrist not zeroed"))),
                () -> zeroed);
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(Radians.of(IntakeConstants.Wrist.ENCODER_OFFSET));
            state = State.IDLE;
            zeroed = true;

            DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);
        });
    }

    public Command retain() {
        return runOnce(() -> {
            motor.setControl(voltReq.withOutput(IntakeConstants.Wrist.RETAINING_VOLTS));
        });
    }

    public void setJiggle(boolean jiggle) {
        if (jiggle) {
            state = State.JIGGLING;
        } else {
            state = State.IDLE;
            motor.setControl(voltReq.withOutput(0));
        }
    }

    public Command eject() {
        return requestPosition(Degrees.of(IntakeConstants.Wrist.EJECT_ANGLE));
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            double manualVolts = joystickPosition.getAsDouble() * IntakeConstants.Wrist.MAX_VOLTS;
            if (manualVolts != 0) {
                state = State.MANUAL;
            } else {
                state = State.IDLE;
            }
            motor.setVoltage(manualVolts);
        });
    }

    @Override
    public void periodic() {
        if (state == State.JIGGLING && zeroed) {
            double phase = Timer.getFPGATimestamp() % IntakeConstants.Wrist.JIGGLE_FREQ_SEC;
            boolean up = phase < (IntakeConstants.Wrist.JIGGLE_FREQ_SEC / 2.0);
            double volts = up ? IntakeConstants.Wrist.UP_VOLTS : IntakeConstants.Wrist.DOWN_VOLTS;
            motor.setControl(voltReq.withOutput(volts));
        } else if(state == State.IDLE) {
            motor.setControl(voltReq.withOutput(IntakeConstants.Wrist.RETAINING_VOLTS));
        }
    }
}
