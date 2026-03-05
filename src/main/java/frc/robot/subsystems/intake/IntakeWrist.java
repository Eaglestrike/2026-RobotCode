package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private static final boolean ENABLE_TUNING = true;

    private enum State {
        IDLE, MOVING_TO_GOAL, MANUAL, JIGGLING
    }

    private final TalonFX motor = new TalonFX(IntakeConstants.Wrist.MOTOR_ID);
    private final PositionTorqueCurrentFOC ctl = new PositionTorqueCurrentFOC(0);

    private State state = State.IDLE;
    private Angle goal = Radians.of(0.0);

    private boolean zeroed = false;

    public IntakeWrist() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Voltage.PeakForwardVoltage = IntakeConstants.Wrist.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -IntakeConstants.Wrist.MAX_VOLTS;
        cfg.Feedback.SensorToMechanismRatio = IntakeConstants.Wrist.GEAR_RATIO;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.Wrist.MAX_ANGLE;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.Wrist.MIN_ANGLE;
        cfg.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.Wrist.MAX_VEL;
        cfg.MotionMagic.MotionMagicAcceleration = IntakeConstants.Wrist.MAX_ACC;

        cfg.Slot0.kP = IntakeConstants.Wrist.kP;
        cfg.Slot0.kI = IntakeConstants.Wrist.kI;
        cfg.Slot0.kD = IntakeConstants.Wrist.kD;
        cfg.Slot0.kS = IntakeConstants.Wrist.kS;
        cfg.Slot0.kV = IntakeConstants.Wrist.kV;
        cfg.Slot0.kG = IntakeConstants.Wrist.kG;
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motor.getConfigurator().apply(cfg);
        motor.setControl(ctl);

        DogLog.tunable("IntakeWrist/RequestDegrees", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Degrees.of(x)));
        });

        DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);

        if (ENABLE_TUNING) {
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
        return runOnce(() -> {
            goal = Radians.of(MathUtil.clamp(position.in(Radians), Math.toRadians(IntakeConstants.Wrist.MIN_ANGLE),
                    Math.toRadians(IntakeConstants.Wrist.MAX_ANGLE)));
            state = State.MOVING_TO_GOAL;
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(Radians.of(IntakeConstants.Wrist.ENCODER_OFFSET));
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
        if (state == State.JIGGLING) {
            double jiggleAngle = (Timer.getFPGATimestamp()
                    % IntakeConstants.Wrist.JIGGLE_FREQ_SEC < IntakeConstants.Wrist.JIGGLE_FREQ_SEC / 2)
                            ? IntakeConstants.Wrist.JIGGLE_UP_ANGLE
                            : IntakeConstants.Wrist.JIGGLE_DOWN_ANGLE;
            goal = Degrees.of(jiggleAngle);
        }

        if (zeroed) {
            motor.setControl(ctl.withPosition(goal));
        } else {
            motor.disable();
        }
    }
}
