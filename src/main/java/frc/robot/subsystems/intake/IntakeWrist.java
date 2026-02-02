// Copy from JerryRoboticsArm2
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder enc;
    private final ProfiledPIDController pid;
    private final ArmFeedforward ff;

    private Angle goal = Radians.of(0.0);

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
        pid.reset(getAngle().magnitude());

        pid.setIntegratorRange(-IntakeConstants.Wrist.IMAX_VOLTS, IntakeConstants.Wrist.IMAX_VOLTS);
        pid.setIZone(Math.toRadians(IntakeConstants.Wrist.IZONE_DEG));

        ff = new ArmFeedforward(
                IntakeConstants.Wrist.kS,
                IntakeConstants.Wrist.kG,
                IntakeConstants.Wrist.kV,
                IntakeConstants.Wrist.kA);

        DogLog.tunable("IntakeWrist/RequestDegrees", 0.0, x -> {
            moveTo(Degrees.of(x)).schedule();
        });
    }

    // encoder gives value in radians
    public Angle getAngle() {
        return Radians.of(enc.getPosition());
    }

    public Command moveTo(Angle position) {
        return runOnce(() -> {
            goal = position;
        });
    }

    public Command zero() {
        return runOnce(() -> {
            System.out.println("Zeroing Intake Wrist");
            motor.getEncoder().setPosition(IntakeConstants.Wrist.ENCODER_OFFSET);
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
        if (!DriverStation.isEnabled()) {
            motor.setVoltage(0.0);
            return;
        }

        goal = Radians.of(MathUtil.clamp(goal.magnitude(), Math.toRadians(IntakeConstants.Wrist.MIN_ANGLE),
                Math.toRadians(IntakeConstants.Wrist.MAX_ANGLE)));
        DogLog.log("IntakeWrist/GoalRad", goal);
        pid.setGoal(goal.magnitude());

        DogLog.log("IntakeWrist/CurrAngDeg", getAngle().in(Degrees), "Degrees");

        double pidVolts = pid.calculate(getAngle().in(Rotations));
        DogLog.log("IntakeWrist/PIDVolts", pidVolts, "Volts");
        double ffVolts = ff.calculate(getAngle().in(Rotations), pid.getSetpoint().velocity);
        DogLog.log("IntakeWrist/FeedforwardVolts", ffVolts, "Volts");
        double volts = pidVolts + ffVolts;
        DogLog.log("IntakeWrist/TotalRequestedVolts", volts, "Volts");
        double outputVolts = MathUtil.clamp(volts, -IntakeConstants.Wrist.MAX_VOLTS, IntakeConstants.Wrist.MAX_VOLTS);
        motor.setVoltage(outputVolts);
        DogLog.log("IntakeWrist/OutputVolts", outputVolts, "Volts");
    }
}
