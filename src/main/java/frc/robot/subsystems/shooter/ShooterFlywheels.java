package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

public class ShooterFlywheels extends SubsystemBase {
    private final SparkFlex motor;
    private final SparkClosedLoopController closedLoopController;
    private SparkFlexConfig config = new SparkFlexConfig();

    public ShooterFlywheels() {
        motor = new SparkFlex(FlywheelConstants.MOTOR_ID, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();

        // Motor configuration
        config.idleMode(IdleMode.kCoast)
                .voltageCompensation(12.0)
                .inverted(false)
                .smartCurrentLimit(FlywheelConstants.CURRENT_LIMIT);

        // Closed loop PID configuration (default feedback sensor is primary encoder)
        config.closedLoop
                .p(FlywheelConstants.kP)
                .i(FlywheelConstants.kI)
                .d(FlywheelConstants.kD)
                .outputRange(FlywheelConstants.MIN_OUTPUT, FlywheelConstants.MAX_OUTPUT);

        // Feedforward configuration
        config.closedLoop.feedForward
                .kS(FlywheelConstants.kS)
                .kV(FlywheelConstants.kV)
                .kA(FlywheelConstants.kA);

        // MAXMotion parameters for velocity control
        config.closedLoop.maxMotion
                .maxAcceleration(FlywheelConstants.MAX_ACCEL);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Tunable parameters
        DogLog.tunable("ShooterFlywheels/kP", FlywheelConstants.kP, newKP -> {
            config.closedLoop.p(newKP);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/kI", FlywheelConstants.kI, newKI -> {
            config.closedLoop.i(newKI);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/kD", FlywheelConstants.kD, newKD -> {
            config.closedLoop.d(newKD);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/kS", FlywheelConstants.kS, newKS -> {
            config.closedLoop.feedForward.kS(newKS);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/kV", FlywheelConstants.kV, newKV -> {
            config.closedLoop.feedForward.kV(newKV);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/kA", FlywheelConstants.kA, newKA -> {
            config.closedLoop.feedForward.kA(newKA);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/maxAccel", FlywheelConstants.MAX_ACCEL, newAccel -> {
            config.closedLoop.maxMotion.maxAcceleration(newAccel);
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        DogLog.tunable("ShooterFlywheels/targetRPM", 0.0, rpm -> {
            startAtVelocity(rpm).schedule();
        });
    }

    /**
     * Run the flywheel at a target velocity using MAXMotion Velocity Control.
     * 
     * @param targetRPM Target velocity in RPM
     */
    public Command startAtVelocity(double targetRPM) {
        return this.runOnce(() -> {
            setVelocity(targetRPM);
        });
    }

    /**
     * Directly set the flywheel velocity (for use by ShooterController)
     * 
     * @param targetRPM Target velocity in RPM
     */
    public void setVelocity(double targetRPM) {
        closedLoopController.setSetpoint(targetRPM, ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Run the flywheel at a target velocity using basic velocity control.
     * 
     * @param targetRPM Target velocity in RPM
     */
    public Command startAtVelocityBasic(double targetRPM) {
        return this.runOnce(() -> {
            closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            motor.stopMotor();
        });
    }

    public double getVelocityRPM() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * Check if the flywheel is at the target velocity within tolerance.
     * 
     * @param targetRPM Target velocity in RPM
     * @return true if within tolerance
     */
    public boolean isAtVelocity(double targetRPM) {
        return Math.abs(getVelocityRPM() - targetRPM) < FlywheelConstants.VELOCITY_TOLERANCE;
    }

    /**
     * Check if the flywheel is at the target velocity within a custom tolerance.
     * 
     * @param targetRPM    Target velocity in RPM
     * @param toleranceRPM Tolerance in RPM
     * @return true if within tolerance
     */
    public boolean isAtVelocity(double targetRPM, double toleranceRPM) {
        return Math.abs(getVelocityRPM() - targetRPM) < toleranceRPM;
    }

    @Override
    public void periodic() {
        DogLog.log("ShooterFlywheels/VelocityRPM", getVelocityRPM(), "RPM");
        DogLog.log("ShooterFlywheels/AppliedOutput", motor.getAppliedOutput());
        DogLog.log("ShooterFlywheels/OutputCurrent", motor.getOutputCurrent(), "Amps");
        DogLog.log("ShooterFlywheels/MotorTemp", motor.getMotorTemperature(), "Celsius");
    }
}
