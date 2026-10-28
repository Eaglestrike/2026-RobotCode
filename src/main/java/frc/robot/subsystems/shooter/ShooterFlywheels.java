package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class ShooterFlywheels extends SubsystemBase {
    public static final boolean ENABLE_TUNING = true;

    private TalonFX motor;
    private VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = config.Slot0;
    private AngularVelocity targetVel = RotationsPerSecond.of(0);

    private final DoubleSubscriber targetAccel = DogLog.tunable("ShooterFlywheels/targetAccel",
            FlywheelConstants.MAX_ACCEL, x -> {
            });

    {
        if (ENABLE_TUNING) {
            DogLog.tunable("ShooterFlywheels/kS", FlywheelConstants.kS, newKS -> {
                slot0Configs.kS = newKS;
                motor.getConfigurator().apply(slot0Configs);
            });

            DogLog.tunable("ShooterFlywheels/kV", FlywheelConstants.kV, newKV -> {
                slot0Configs.kV = newKV;
                motor.getConfigurator().apply(slot0Configs);
            });

            DogLog.tunable("ShooterFlywheels/kA", FlywheelConstants.kA, newKA -> {
                slot0Configs.kA = newKA;
                motor.getConfigurator().apply(slot0Configs);
            });

            DogLog.tunable("ShooterFlywheels/kP", FlywheelConstants.kP, newKP -> {
                slot0Configs.kP = newKP;
                motor.getConfigurator().apply(slot0Configs);
            });

            DogLog.tunable("ShooterFlywheels/kI", FlywheelConstants.kI, newKI -> {
                slot0Configs.kI = newKI;
                motor.getConfigurator().apply(slot0Configs);
            });

            DogLog.tunable("ShooterFlywheels/kD", FlywheelConstants.kD, newKD -> {
                slot0Configs.kD = newKD;
                motor.getConfigurator().apply(slot0Configs);
            });
        }
    }

    public ShooterFlywheels() {
        VoltageConfigs voltages = config.Voltage;
        voltages.PeakForwardVoltage = FlywheelConstants.MAX_VOLTS;
        voltages.PeakReverseVoltage = -FlywheelConstants.MAX_VOLTS;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = FlywheelConstants.GEAR_RATIO;

        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        slot0Configs.kS = FlywheelConstants.kS;
        slot0Configs.kV = FlywheelConstants.kV;
        slot0Configs.kA = FlywheelConstants.kA;
        slot0Configs.kP = FlywheelConstants.kP;
        slot0Configs.kI = FlywheelConstants.kI;
        slot0Configs.kD = FlywheelConstants.kD;

        motor = new TalonFX(FlywheelConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        motor.getConfigurator().apply(config);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getPosition().setUpdateFrequency(10);
        motor.optimizeBusUtilization();

        DogLog.tunable("ShooterFlywheels/Request RPM", 0.0, x -> {
            requestVelocity(x);
        });
    }

    /**
     * Directly set the flywheel target velocity (for use by Shooter in periodic())
     * 
     * @param vel Target velocity in rotations per minute
     */
    public void requestVelocity(double vel) {
        vel /= 60;
        this.targetVel = RotationsPerSecond.of(vel);
        if (Math.abs(vel - FlywheelConstants.IDLE_VELOCITY / 60) < 0.1) {
            motor.setVoltage(FlywheelConstants.IDLE_VOLTAGE);
        } else if (motor.getVelocity().getValue().isNear(
                targetVel, RotationsPerSecond.of(ShooterConstants.FlywheelConstants.VELOCITY_TOLERANCE))) {
            // motor.setControl(velocityRequest.withVelocity(this.targetVel).withAcceleration(targetAccel.get()));
            motor.setControl(velocityRequest.withVelocity(this.targetVel));
        } else if (vel < motor.getVelocity().getValueAsDouble()) {
            // Coast until just before the target if trying to reduce speed
            motor.setVoltage(0);
        } else {
            // motor.setControl(velocityRequest.withVelocity(this.targetVel).withAcceleration(targetAccel.get()));
            motor.setControl(velocityRequest.withVelocity(this.targetVel));
        }
    }

    public void setManualInput(double joystickPosition) {
        motor.setVoltage(joystickPosition * FlywheelConstants.MAX_VOLTS);
    }

    public boolean atVelocity() {
        return motor.getVelocity().getValue()
                .isNear(targetVel, RotationsPerSecond.of(FlywheelConstants.VELOCITY_TOLERANCE));
    }

    public Command stop() {
        return this.runOnce(() -> {
            motor.stopMotor();
        });
    }

    @Override
    public void periodic() {
        DogLog.log("ShooterFlywheels/Velocity_rps", motor.getVelocity().getValueAsDouble());
        DogLog.log("ShooterFlywheels/TargetVelocity_rps", targetVel.in(RotationsPerSecond));
    }
}