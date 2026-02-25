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
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class ShooterFlywheels extends SubsystemBase {
    public static final boolean ENABLE_TUNING = true;

    private TalonFX motor;
    private VelocityTorqueCurrentFOC motionMagic = new VelocityTorqueCurrentFOC(0);
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = config.Slot0;
    private AngularVelocity targetVel = AngularVelocity.ofBaseUnits(0, RotationsPerSecond);

    private final DoubleSubscriber targetAccel = DogLog.tunable("ShooterFlywheels/targetAccel",
            FlywheelConstants.MAX_ACCEL, newTargetAccel -> {
                slot0Configs.kD = newTargetAccel;
                motor.getConfigurator().apply(slot0Configs);
            });

    {
        if(ENABLE_TUNING) {
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

        slot0Configs.kS = FlywheelConstants.kS;
        slot0Configs.kV = FlywheelConstants.kV;
        slot0Configs.kA = FlywheelConstants.kA;
        slot0Configs.kP = FlywheelConstants.kP;
        slot0Configs.kI = FlywheelConstants.kI;
        slot0Configs.kD = FlywheelConstants.kD;

        motor = new TalonFX(FlywheelConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        motor.getConfigurator().apply(config);
    }

    public Command startAtVelocity(double targetVel) {
        return this.runOnce(() -> {
            this.targetVel = AngularVelocity.ofBaseUnits(targetVel, RotationsPerSecond);
            motor.setControl(motionMagic.withVelocity(this.targetVel).withAcceleration(targetAccel.get()));
        });
    }

    /**
     * Function to set flywheel velocity
     * @param targetVel in RPM
     */
    public void setVelocity(double targetVel) {
        this.targetVel = AngularVelocity.ofBaseUnits(targetVel / 60, RotationsPerSecond);
        motor.setControl(motionMagic.withVelocity(this.targetVel).withAcceleration(targetAccel.get()));
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return this.run(() -> {
            motor.setVoltage(joystickPosition.getAsDouble() * FlywheelConstants.MAX_VOLTS);
        });
    }

    public boolean atVelocity() {
        return motor.getVelocity().getValue()
            .isNear(targetVel, AngularVelocity.ofBaseUnits(FlywheelConstants.VELOCITY_TOLERANCE, RotationsPerSecond));
    }

    public Command stop() {
        return this.runOnce(() -> {
            motor.stopMotor();
        });
    }
}