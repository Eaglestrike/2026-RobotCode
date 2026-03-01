// Copy from JerryRoboticsArm2
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    // public boolean hasExtendedHopper = false;

    private enum State {
        IDLE, MOVING_TO_GOAL, MANUAL, JIGGLING
    }

    private final SparkMax motor = new SparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder enc;
    private final ProfiledPIDController pid;
    private final ArmFeedforward ff;

    private State state = State.IDLE;
    private Angle goal = Radians.of(0.0);

    private boolean zeroed = false;

    public IntakeWrist() {
        enc = motor.getEncoder();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        cfg.encoder.positionConversionFactor(2.0 * Math.PI / IntakeConstants.Wrist.GEAR_RATIO);
        cfg.encoder.velocityConversionFactor(2.0 * Math.PI / IntakeConstants.Wrist.GEAR_RATIO / 60.0);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = new ProfiledPIDController(
                IntakeConstants.Wrist.kP,
                IntakeConstants.Wrist.kI,
                IntakeConstants.Wrist.kD,
                new TrapezoidProfile.Constraints(
                        IntakeConstants.Wrist.MAX_VEL,
                        IntakeConstants.Wrist.MAX_ACC));
        pid.setTolerance(Math.toRadians(1.0));
        pid.reset(getAngle().in(Radians));

        pid.setIntegratorRange(-IntakeConstants.Wrist.IMAX_VOLTS, IntakeConstants.Wrist.IMAX_VOLTS);
        pid.setIZone(Math.toRadians(IntakeConstants.Wrist.IZONE_DEG));

        ff = new ArmFeedforward(
                IntakeConstants.Wrist.kS,
                IntakeConstants.Wrist.kG,
                IntakeConstants.Wrist.kV,
                IntakeConstants.Wrist.kA);

        DogLog.tunable("IntakeWrist/RequestDegrees", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Degrees.of(x)));
        });

        DogLog.log("IntakeWrist/IntakeWristZeroed", zeroed);
    }

    // encoder gives value in radians
    public Angle getAngle() {
        return Radians.of(enc.getPosition());
    }

    public boolean clearedHopper() {
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
            motor.getEncoder().setPosition(IntakeConstants.Wrist.ENCODER_OFFSET);
            goal = Radians.of(motor.getEncoder().getPosition()); // So the robot doesn't automatically move when enabling
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
        DogLog.log("IntakeWrist/CurrAngDeg", getAngle().in(Degrees), "Degrees");

        if (!DriverStation.isEnabled() || !zeroed) {
            motor.setVoltage(0.0);
            return;
        }

        if (state == State.JIGGLING) {
            double jiggleAngle = (Timer.getFPGATimestamp()
                    % IntakeConstants.Wrist.JIGGLE_FREQ_SEC < IntakeConstants.Wrist.JIGGLE_FREQ_SEC / 2)
                            ? IntakeConstants.Wrist.JIGGLE_UP_ANGLE
                            : IntakeConstants.Wrist.JIGGLE_DOWN_ANGLE;
            goal = Degrees.of(jiggleAngle);
            pid.setGoal(goal.in(Radians));
            // DogLog.log("IntakeWrist/GoalRad", goal);
        } else {
            // DogLog.log("IntakeWrist/GoalRad", goal);
            pid.setGoal(goal.in(Radians));
        }

        double pidVolts = pid.calculate(getAngle().in(Radians));
        // DogLog.log("IntakeWrist/PIDVolts", pidVolts, "Volts");
        double ffVolts = ff.calculate(getAngle().in(Radians), pid.getSetpoint().velocity);
        // DogLog.log("IntakeWrist/FeedforwardVolts", ffVolts, "Volts");
        double volts = pidVolts + ffVolts;
        // DogLog.log("IntakeWrist/TotalRequestedVolts", volts, "Volts");
        double maxVolts = IntakeConstants.Wrist.EXTEND_HOPPER_VOLTS;
        // README: do NOT uncomment this unless you want to break our robot.
        // if (this.clearedHopper()) {
        // hasExtendedHopper = true;
        // }
        // if (!hasExtendedHopper) {
        // maxVolts = IntakeConstants.Wrist.EXTEND_HOPPER_VOLTS;
        // }
        double outputVolts = MathUtil.clamp(volts, -maxVolts, maxVolts);
        if (state == State.MOVING_TO_GOAL || state == State.JIGGLING) {
            motor.setVoltage(outputVolts);
        }
        // DogLog.log("IntakeWrist/OutputVolts", outputVolts, "Volts");
    }
}
