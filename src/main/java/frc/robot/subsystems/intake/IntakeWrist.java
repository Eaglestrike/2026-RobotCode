package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController ctl = motor.getClosedLoopController();
    private final SparkFlexConfig cfg = new SparkFlexConfig();

    private final RelativeEncoder enc = motor.getEncoder();
    private final ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(
            IntakeConstants.Wrist.MAX_VEL,
            IntakeConstants.Wrist.MAX_ACC));

    private final DoubleSubscriber kP = DogLog.tunable("IntakeWrist/kP", 0d, (x) -> {
        cfg.closedLoop.p(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kI = DogLog.tunable("IntakeWrist/kI", 0d, (x) -> {
        cfg.closedLoop.i(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kD = DogLog.tunable("IntakeWrist/kD", 0d, (x) -> {
        cfg.closedLoop.d(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kS = DogLog.tunable("IntakeWrist/kS", 0d, (x) -> {
        cfg.closedLoop.feedForward.kS(0);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kA = DogLog.tunable("IntakeWrist/kA", 0d, (x) -> {
        cfg.closedLoop.feedForward.kA(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kV = DogLog.tunable("IntakeWrist/kV", 0d, (x) -> {
        cfg.closedLoop.feedForward.kV(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kCos = DogLog.tunable("IntakeWrist/kCos", 0d, (x) -> {
        cfg.closedLoop.feedForward.kCos(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });
    private final DoubleSubscriber kCosRatio = DogLog.tunable("IntakeWrist/kCosRatio", 0d, (x) -> {
        cfg.closedLoop.feedForward.kCosRatio(x);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    public IntakeWrist() {
        cfg
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);

        cfg.encoder
                .positionConversionFactor(2.0 * Math.PI / IntakeConstants.Wrist.GEAR_RATIO)
                .velocityConversionFactor(2.0 * Math.PI / IntakeConstants.Wrist.GEAR_RATIO / 60.0);

        cfg.closedLoop
                .iZone(IntakeConstants.Wrist.IZONE.in(Rotations))
                .iMaxAccum(IntakeConstants.Wrist.IMAX_VOLTS).maxMotion
                .maxAcceleration(IntakeConstants.Wrist.MAX_ACC)
                .cruiseVelocity(IntakeConstants.Wrist.MAX_VEL);

        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pid.setTolerance(IntakeConstants.Wrist.TOLERANCE.in(Radians));

        DogLog.tunable("IntakeWrist/RequestDegrees", 0.0, x -> {
            CommandScheduler.getInstance().schedule(moveTo(Degrees.of(x)));
        });
    }

    // encoder gives value in radians
    public Angle getAngle() {
        return Radians.of(enc.getPosition());
    }

    public Command moveTo(Angle position) {
        return runOnce(() -> {
            double rad = MathUtil.clamp(position.in(Radians), IntakeConstants.Wrist.MIN_ANGLE.in(Rotations),
                    IntakeConstants.Wrist.MAX_ANGLE.in(Radians));
            ctl.setSetpoint(rad, ControlType.kMAXMotionPositionControl);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            System.out.println("Zeroing Intake Wrist");
            motor.getEncoder().setPosition(IntakeConstants.Wrist.ENCODER_OFFSET.in(Rotations));
            ctl.setSetpoint(IntakeConstants.Wrist.ENCODER_OFFSET.in(Rotations), ControlType.kMAXMotionPositionControl);
        });
    }

    public Command kill() {
        return runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    public Command eject() {
        return runOnce(() -> {
            this.moveTo(IntakeConstants.Wrist.EJECT_ANGLE);
        });
    }

    public Command setManualVoltage(double joystickPosition) {
        return run(() -> {
            motor.setVoltage(joystickPosition * IntakeConstants.Wrist.MAX_VOLTS);
        });
    }

    @Override
    public void periodic() {
        if (!DriverStation.isEnabled()) {
            motor.setVoltage(0.0);
            return;
        }
    }
}
