package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

public class ShooterHood extends SubsystemBase {
    private final SparkMax motor = new SparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder enc;
    private final ProfiledPIDController pid;
    private final ArmFeedforward ff;

    private Angle goal = Radians.of(0.0);

    public ShooterHood() {
        enc = motor.getEncoder();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        cfg.encoder.positionConversionFactor(2.0 * Math.PI / HoodConstants.GEAR_RATIO);
        cfg.encoder.velocityConversionFactor(2.0 * Math.PI / HoodConstants.GEAR_RATIO / 60.0);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = new ProfiledPIDController(
                HoodConstants.kP,
                HoodConstants.kI,
                HoodConstants.kD,
                new TrapezoidProfile.Constraints(
                        HoodConstants.MAX_VEL,
                        HoodConstants.MAX_ACC));
        pid.setTolerance(Math.toRadians(1.0));
        pid.reset(getAngle().magnitude());

        pid.setIntegratorRange(-HoodConstants.IMAX_VOLTS, HoodConstants.IMAX_VOLTS);
        pid.setIZone(Math.toRadians(HoodConstants.IZONE_DEG));

        ff = new ArmFeedforward(
                HoodConstants.kS,
                HoodConstants.kG,
                HoodConstants.kV,
                HoodConstants.kA);

        DogLog.tunable("ShooterHood/RequestDegrees", 0.0, x -> {
            goToPos(Degrees.of(x)).schedule();
        });
    }

    // encoder gives value in radians
    public Angle getAngle() {
        return Radians.of(enc.getPosition());
    }

    public Command goToPos(Angle position) {
        return runOnce(() -> {
            setGoal(position);
        });
    }

    /**
     * Directly set the hood goal position (for use by ShooterController)
     */
    public void setGoal(Angle position) {
        goal = position;
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            System.out.println("Zeroing Shooter Hood");
            motor.getEncoder().setPosition(HoodConstants.ENCODER_OFFSET);
        });
    }

    public Command setManualVoltage(double joystickPosition) {
        return runOnce(() -> {
            motor.setVoltage(joystickPosition * HoodConstants.MAX_VOLTS);
        });
    }

    @Override
    public void periodic() {
        if (!DriverStation.isEnabled()) {
            motor.setVoltage(0.0);
            return;
        }

        goal = Radians.of(MathUtil.clamp(goal.magnitude(), Math.toRadians(HoodConstants.MIN_ANGLE),
                Math.toRadians(HoodConstants.MAX_ANGLE)));
        DogLog.log("ShooterHood/GoalRad", goal);
        pid.setGoal(goal.magnitude());

        DogLog.log("ShooterHood/CurrAngDeg", getAngle().in(Degrees), "Degrees");

        double pidVolts = pid.calculate(getAngle().in(Rotations));
        DogLog.log("ShooterHood/PIDVolts", pidVolts, "Volts");
        double ffVolts = ff.calculate(getAngle().in(Rotations), pid.getSetpoint().velocity);
        DogLog.log("ShooterHood/FeedforwardVolts", ffVolts, "Volts");
        double volts = pidVolts + ffVolts;
        DogLog.log("ShooterHood/TotalRequestedVolts", volts, "Volts");
        double outputVolts = MathUtil.clamp(volts, -HoodConstants.MAX_VOLTS, HoodConstants.MAX_VOLTS);
        motor.setVoltage(outputVolts);
        DogLog.log("ShooterHood/OutputVolts", outputVolts, "Volts");
    }

    public Pose3d getComponentPose3d(Pose3d turretPose) {
        // offset of the hood relative to the turret frame (forward, lateral, up)
        double offsetX = 0.106;
        double offsetY = 0.0;

        var yaw = turretPose.getRotation().getZ();

        // rotate the XY offset by the turret yaw to transform into field coordinates
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double fieldOffsetX = cos * offsetX - sin * offsetY;
        double fieldOffsetY = sin * offsetX + cos * offsetY;

        return new Pose3d(
                turretPose.getX() + fieldOffsetX,
                turretPose.getY() + fieldOffsetY,
                turretPose.getZ() + 0.0713,
                new Rotation3d(0, getAngle().in(Radians), yaw));
    }
}
