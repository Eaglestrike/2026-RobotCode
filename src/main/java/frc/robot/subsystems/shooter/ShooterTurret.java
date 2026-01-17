package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class ShooterTurret extends SubsystemBase {
    private TalonFX motor;
    private CANcoder cantCoder;
    private MotionMagicVoltage position = new MotionMagicVoltage(0).withSlot(0);
    private Slot0Configs slot0Configs;
    private EagleSwerveDrivetrain drive;

    /**
     * Maximum rotation in radians
     * TODO: if this is >360 degrees, implement proper wrapping in {@link #setFacing}!!!
     */
    private final Angle maxRotation = Radians.of(2);

    private final DoubleSubscriber kP = DogLog.tunable("ShooterTurret/kP", 0.0, x -> {
        slot0Configs.kP = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kI = DogLog.tunable("ShooterTurret/kI", 0.0, x -> {
        slot0Configs.kI = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kD = DogLog.tunable("ShooterTurret/kD", 0.0, x -> {
        slot0Configs.kD = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kV = DogLog.tunable("ShooterTurret/kV", 0.0, x -> {
        slot0Configs.kV = x;
        motor.getConfigurator().apply(slot0Configs);
    });
    private final DoubleSubscriber kS = DogLog.tunable("ShooterTurret/kS", 0.0, x -> {
        slot0Configs.kS = x;
        motor.getConfigurator().apply(slot0Configs);
    });

    public ShooterTurret(EagleSwerveDrivetrain drive) {
        this.drive = drive;
        this.motor = new TalonFX(0);
        this.cantCoder = new CANcoder(0);
        var cfg = new TalonFXConfiguration();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = this.cantCoder.getDeviceID();
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = this.maxRotation.in(Rotations);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        this.motor.getConfigurator().apply(cfg);
    }

    public Command requestPosition(Angle pos) {
        if (pos.lt(Radians.of(0)) || pos.gt(this.maxRotation)) {
            DogLog.logFault(String.format("[ShooterTurret] Invalid position (%.2f) requested >:(", pos));
            return runOnce(() -> {
            });
        } else {
            return runOnce(() -> {
                this.motor.setControl(this.position.withPosition(pos));
            });
        }
    }

    public Command setFacing(Pose2d to) {
        return run(() -> {
            var pose = drive.getState().Pose.plus(new Transform2d(ShooterConstants.OFFSET, new Rotation2d()));
            var angle = to.getTranslation().minus(pose.getTranslation()).getAngle()
                    .minus(pose.getRotation());

            this.motor.setControl(this.position.withPosition(angle.getRotations()));
        });
    }

    public StatusSignal<Angle> getAngle() {
        return this.cantCoder.getAbsolutePosition();
    }

    public Command stop() {
        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }
}
