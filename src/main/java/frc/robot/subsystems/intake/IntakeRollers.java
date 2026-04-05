package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;


public class IntakeRollers extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFX followerMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(6.0);

    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.Rollers.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        followerMotor = new TalonFX(IntakeConstants.Rollers.FOLLOWER_MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        var cfg = motor.getConfigurator();
        var currentCfg = new CurrentLimitsConfigs();
        currentCfg.StatorCurrentLimit = IntakeConstants.Rollers.CURRENT_LIMIT;
        currentCfg.StatorCurrentLimitEnable = true;
        cfg.apply(currentCfg);
        motor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();

        Trigger currentLimitTrigger = new Trigger(() -> motor.getStatorCurrent().getValueAsDouble() > IntakeConstants.Rollers.STALL_THRESHOLD);
        currentLimitTrigger.onTrue(stop());
    }

    public Command start() {
        return run(() -> {
            double limitedVoltage = slewRateLimiter.calculate(IntakeConstants.Rollers.VOLTS);
            motor.setControl(voltageRequest.withOutput(limitedVoltage));
            followerMotor.setControl(voltageRequest.withOutput(limitedVoltage));
        }).until(() -> Math.abs(slewRateLimiter.lastValue() - IntakeConstants.Rollers.VOLTS) < 0.1)
                .beforeStarting(() -> slewRateLimiter.reset(0));
    }

    public Command eject() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.EJECT_VOLTS));
            followerMotor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.EJECT_VOLTS));
        });
    }

    public Command jiggle() {
        return run(() -> {
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.JIGGLE_VOLTS));
            followerMotor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.JIGGLE_VOLTS));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(0));
            followerMotor.setControl(voltageRequest.withOutput(0));
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setControl(voltageRequest.withOutput(joystickPosition.getAsDouble() * IntakeConstants.Rollers.VOLTS));
            followerMotor.setControl(voltageRequest.withOutput(joystickPosition.getAsDouble() * IntakeConstants.Rollers.VOLTS));
        });
    }
}
