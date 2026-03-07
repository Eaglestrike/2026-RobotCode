package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class IntakeWrist extends SubsystemBase {
    private static final boolean ENABLE_TUNING = false;
    private static final boolean BANG_BANG = true;

    private enum State {
        IDLE, MOVING_TO_GOAL, MANUAL, JIGGLING
    }

    private final TalonFX motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
    private final PositionTorqueCurrentFOC ctl = new PositionTorqueCurrentFOC(0);
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
        if(BANG_BANG) {
            cfg.Voltage.PeakForwardVoltage = IntakeConstants.Wrist.MAX_VOLTS;
            cfg.Voltage.PeakReverseVoltage = -IntakeConstants.Wrist.MAX_VOLTS;
        } else {
            cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.Wrist.MAX_ANGLE;
            cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.Wrist.MIN_ANGLE;
            cfg.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.Wrist.MAX_VEL;
            cfg.MotionMagic.MotionMagicAcceleration = IntakeConstants.Wrist.MAX_ACC;
        }
       
        cfg.Slot0.kP = IntakeConstants.Wrist.kP;
        cfg.Slot0.kI = IntakeConstants.Wrist.kI;
        cfg.Slot0.kD = IntakeConstants.Wrist.kD;
        cfg.Slot0.kS = IntakeConstants.Wrist.kS;
        cfg.Slot0.kV = IntakeConstants.Wrist.kV;
        cfg.Slot0.kG = IntakeConstants.Wrist.kG;
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motor.getConfigurator().apply(cfg);
        motor.setControl(BANG_BANG ? voltReq : ctl);

        DogLog.tunable("IntakeWrist/RequestDegrees", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Degrees.of(x)));
        });


        DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);

        if (ENABLE_TUNING) {
            DogLog.tunable("IntakeWrist/EncoderOffset", offset, x -> {
                offset = x;
            });

            DogLog.tunable("IntakeWrist/kP", cfg.Slot0.kP, (x) -> {
                cfg.Slot0.kP = x;
                motor.getConfigurator().apply(cfg);
            });
            DogLog.tunable("IntakeWrist/kI", cfg.Slot0.kI, (x) -> {
                cfg.Slot0.kI = x;
                motor.getConfigurator().apply(cfg);
            });
            DogLog.tunable("IntakeWrist/kD", cfg.Slot0.kD, (x) -> {
                cfg.Slot0.kD = x;
                motor.getConfigurator().apply(cfg);
            });
            DogLog.tunable("IntakeWrist/kS", cfg.Slot0.kS, (x) -> {
                cfg.Slot0.kS = x;
                motor.getConfigurator().apply(cfg);
            });
            DogLog.tunable("IntakeWrist/kV", cfg.Slot0.kV, (x) -> {
                cfg.Slot0.kV = x;
                motor.getConfigurator().apply(cfg);
            });
            DogLog.tunable("IntakeWrist/kG", cfg.Slot0.kG, (x) -> {
                cfg.Slot0.kG = x;
                motor.getConfigurator().apply(cfg);
            });
            DogLog.tunable("IntakeWrist/kA", cfg.Slot0.kA, (x) -> {
                cfg.Slot0.kA = x;
                motor.getConfigurator().apply(cfg);
            });
        }
    }

    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    public boolean hopperExtended() {
        // todo: adjust this to be a specific position
        return getAngle().in(Radians) > 0;
    }

    public Command requestPosition(Angle position) {
        if(BANG_BANG) {
            if(!zeroed) {
                return Commands.print("Wrist not zeroed");
            }
            double voltage = position.isNear(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE), 0.5) ? IntakeConstants.Wrist.UP_VOLTS : IntakeConstants.Wrist.DOWN_VOLTS;
            double stallCurrent = position.isNear(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE), 0.5) ? IntakeConstants.Wrist.UP_STALL_CURRENT : IntakeConstants.Wrist.DOWN_STALL_CURRENT;
            DogLog.log("Intake/Intake voltage", voltage);
            DogLog.log("Intake/stall current", stallCurrent);
            return new SequentialCommandGroup(
                runOnce(() -> motor.setControl(voltReq.withOutput(voltage))),
                new WaitUntilCommand(() -> motor.getSupplyCurrent().getValueAsDouble() > stallCurrent),
                runOnce(() -> {
                    motor.setControl(voltReq.withOutput(0));
                    motor.setPosition(position); // Cos stalling messes up the encoder
                })
            ).finallyDo(() -> motor.setControl(voltReq.withOutput(0)));
        }

        return runOnce(() -> {
            goal = Radians.of(MathUtil.clamp(position.in(Radians), Math.toRadians(IntakeConstants.Wrist.MIN_ANGLE),
                    Math.toRadians(IntakeConstants.Wrist.MAX_ANGLE)));
            state = State.MOVING_TO_GOAL;
        });
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
        return runOnce(() -> {
            state = State.JIGGLING;
        });
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
        if(ENABLE_TUNING) {
            DogLog.log("IntakeWrist/Position", motor.getPosition().getValueAsDouble());
        }
            
        if (state == State.JIGGLING) {
            double jiggleAngle = (Timer.getFPGATimestamp()
                    % IntakeConstants.Wrist.JIGGLE_FREQ_SEC < IntakeConstants.Wrist.JIGGLE_FREQ_SEC / 2)
                            ? IntakeConstants.Wrist.JIGGLE_UP_ANGLE
                            : IntakeConstants.Wrist.JIGGLE_DOWN_ANGLE;
            goal = Degrees.of(jiggleAngle);
        }

        if (zeroed && !BANG_BANG) {
            motor.setControl(ctl.withPosition(goal));
        } else if(!BANG_BANG){
            motor.disable();
        }
    }
}
