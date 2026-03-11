package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;


public class IntakeWrist extends SubsystemBase {
    private enum State {
        IDLE, MOVING_TO_GOAL, MANUAL, JIGGLING
    }

    private final TalonFX motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final VoltageOut voltReq = new VoltageOut(0).withEnableFOC(true);

    private State state = State.IDLE;
    private Angle goal = Radians.of(0.0);
    private double offset = IntakeConstants.Wrist.ENCODER_OFFSET;

    private boolean zeroed = false;

    public IntakeWrist() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Voltage.PeakForwardVoltage = IntakeConstants.Wrist.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -IntakeConstants.Wrist.MAX_VOLTS;
        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.Wrist.GEAR_RATIO;

        cfg.Slot0.kP = IntakeConstants.Wrist.kP;
        cfg.Slot0.kI = IntakeConstants.Wrist.kI;
        cfg.Slot0.kD = IntakeConstants.Wrist.kD;
        cfg.Slot0.kS = IntakeConstants.Wrist.kS;
        cfg.Slot0.kV = IntakeConstants.Wrist.kV;
        cfg.Slot0.kG = IntakeConstants.Wrist.kG;
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motor.getConfigurator().apply(cfg);
        motor.setControl(voltReq);

        DogLog.tunable("IntakeWrist/RequestDegrees", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Degrees.of(x)));
        });

        DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);
    }

    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    public boolean hopperExtended() {
        // todo: adjust this to be a specific position
        return getAngle().in(Radians) > 0;
    }

    public Command requestPosition(Angle position) {
            double voltage = position.isNear(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE), 0.5) ? IntakeConstants.Wrist.UP_VOLTS : IntakeConstants.Wrist.DOWN_VOLTS;
            double stallCurrent = position.isNear(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE), 0.5) ? IntakeConstants.Wrist.UP_STALL_CURRENT : IntakeConstants.Wrist.DOWN_STALL_CURRENT;
            DogLog.log("Intake/Intake voltage", voltage);
            DogLog.log("Intake/stall current", stallCurrent);
            return Commands.select(
                    Map.ofEntries(
                            Map.entry(true,
                                    new SequentialCommandGroup(
                                            runOnce(() -> motor.setControl(voltReq.withOutput(voltage))),
                                            new WaitUntilCommand(() -> motor.getSupplyCurrent().getValueAsDouble() > stallCurrent),
                                            runOnce(() -> {
                                                motor.setControl(voltReq.withOutput(IntakeConstants.Wrist.RETAINING_VOLTS));
                                                motor.setPosition(position); // Cos stalling messes up the encoder
                                            })
                                    ).finallyDo(() -> motor.setControl(voltReq.withOutput(0)))),
                            Map.entry(false, Commands.print("Wrist not zeroed"))),
                    () -> zeroed);
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(Radians.of(offset));
            state = State.IDLE;
            zeroed = true;

            DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);
        });
    }

    public Command jiggle() {
        return Commands.repeatingSequence(
                Commands.race(
                        requestPosition(Degrees.of(IntakeConstants.Wrist.MAX_ANGLE)),
                        Commands.waitSeconds(IntakeConstants.Wrist.JIGGLE_FREQ_SEC)),

                Commands.race(requestPosition(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE)),
                        Commands.waitSeconds(IntakeConstants.Wrist.JIGGLE_FREQ_SEC)));
    }

    public Command kill() {
        return runOnce(() -> {
            motor.setVoltage(0);
            state = State.IDLE;
        });
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
    }
}
