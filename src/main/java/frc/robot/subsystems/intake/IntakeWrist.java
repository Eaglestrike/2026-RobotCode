package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class IntakeWrist extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder enc;
    private final ProfiledPIDController pid;
    private final double baseMinDeg;
    private final double baseMaxDeg;

    private double prevSpVel = 0.0;
    private double lastOutVolts = 0.0;
    private double lastDir = 0.0;
    private final double accelLimitRadPerS2;
    private boolean isEnabled = false;

    private double goalDeg = 0.0;

    public IntakeWrist() {
        this.baseMinDeg = IntakeConstants.Wrist.MIN_ANGLE;
        this.baseMaxDeg = IntakeConstants.Wrist.MAX_ANGLE;
        this.accelLimitRadPerS2 = Math.max(0.0, IntakeConstants.Wrist.MAX_ACC);
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
        pid.reset(getAngleRad());

        pid.setIntegratorRange(-IntakeConstants.Wrist.IMAX_VOLTS, IntakeConstants.Wrist.IMAX_VOLTS);
        pid.setIZone(Math.toRadians(IntakeConstants.Wrist.IZONE_DEG));

        DogLog.tunable("IntakeWristRequest", 0.0, x -> {
            moveTo(Degrees.of(x)).schedule();
        });

        RobotModeTriggers.teleop().onTrue(runOnce(() -> {
            System.out.println("enabled");
            isEnabled = true;
        }).ignoringDisable(true));
        RobotModeTriggers.disabled().onTrue(runOnce(() -> {
            System.out.println("disabled");
            isEnabled = false;
        }).ignoringDisable(true));
    }

    public double getAngleRad() {
        return enc.getPosition();
    }

    public double getAngleDeg() {
        return Math.toDegrees(getAngleRad());
    }

    public Command moveTo(Angle position) {
        return runOnce(() -> {
            DogLog.log("IntakeWrist/RequestedPositionDeg", position.in(Degrees), "Radians");
            goalDeg = position.in(Degrees);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.getEncoder().setPosition(IntakeConstants.Wrist.ENCODER_OFFSET_RAD);
        });
    }

    public Command kill() {
        return runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    public Command setManualVoltage(double joystickPosition) {
        return runOnce(() -> {
            motor.setVoltage(joystickPosition * IntakeConstants.Wrist.MAX_VOLTS);
        });
    }

    @Override
    public void periodic() {
        // Apply joint angle limits
        DogLog.log("IntakeWrist/GoalDeg", goalDeg, "Degrees");
        DogLog.log("IntakeWrist/Enabled", isEnabled);
        double localGoalDeg = goalDeg;
        localGoalDeg = MathUtil.clamp(localGoalDeg, baseMinDeg, baseMaxDeg);
        pid.setGoal(Math.toRadians(localGoalDeg));

        // Setpoint telemetry (50Hz)
        TrapezoidProfile.State sp = pid.getSetpoint();

        if (!isEnabled) {
            motor.setVoltage(0.0);
            return;
        }

        // Control (reuse sp from telemetry above)
        double accel = (sp.velocity - prevSpVel) / 0.02;
        if (accelLimitRadPerS2 > 0.0) {
            // Finite-difference accel is noisy; clamp to something sane (profile
            // constraint).
            accel = MathUtil.clamp(accel, -accelLimitRadPerS2, accelLimitRadPerS2);
        }
        prevSpVel = sp.velocity;
        double vel = sp.velocity;
        // Robust direction for Ks/Ka near stop:
        // - Prefer desired velocity sign when moving
        // - Otherwise use goal - setpoint position, but with a small deadband to avoid
        // sign flapping
        // - Otherwise hold last non-zero direction (prevents +/-Ks jitter around 0)
        double posErr = pid.getGoal().position - sp.position;
        double sign;
        if (Math.abs(vel) > 1e-4) {
            sign = Math.signum(vel);
        } else if (Math.abs(posErr) > Math.toRadians(0.25)) {
            sign = Math.signum(posErr);
        } else {
            sign = lastDir;
        }
        if (sign != 0.0)
            lastDir = sign;

        // Gravity feedforward for arm-like joints.
        // IMPORTANT: Gravity depends on *actual* angle, not setpoint. Using setpoint
        // can cause brief pulses when
        // the setpoint jumps (e.g., target=90deg => cos=0) even if the arm hasn't moved
        // yet.
        double selfAngleRad = getAngleRad();
        DogLog.log("IntakeWrist/CurrAng", selfAngleRad);
        double gravAngleRad = selfAngleRad;
        double gVolts = IntakeConstants.Wrist.kG * Math.cos(gravAngleRad);

        double ffVolts = IntakeConstants.Wrist.kS * sign
                + gVolts
                + IntakeConstants.Wrist.kV * vel
                + IntakeConstants.Wrist.kA * Math.abs(accel) * sign;
        DogLog.log("IntakeWrist/FeedforwardVolts", ffVolts, "Volts");
        double pidVolts = pid.calculate(getAngleRad());
        DogLog.log("IntakeWrist/PIDVolts", pidVolts, "Volts");
        double volts = pidVolts + ffVolts;
        DogLog.log("IntakeWrist/TotalVolts", volts, "Volts");
        lastOutVolts = MathUtil.clamp(volts, -IntakeConstants.Wrist.MAX_VOLTS, IntakeConstants.Wrist.MAX_VOLTS);
        motor.setVoltage(lastOutVolts);
        DogLog.log("IntakeWrist/OutputVolts", lastOutVolts, "Volts");
    }
}
