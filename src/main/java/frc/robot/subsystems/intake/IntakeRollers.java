package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

import org.jspecify.annotations.Nullable;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private @Nullable Timer autoejectTrigger = null;
    private boolean intaking = false;
    private boolean autoejecting = false;
    private final boolean AUTOEJECT_ENABLED = true;

    public IntakeRollers() {
        motor = new TalonFX(IntakeConstants.Rollers.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        motor.optimizeBusUtilization();
    }

    public Command start() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.VOLTS));
            this.intaking = true;
        });
    }

    public Command eject() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(-IntakeConstants.Rollers.EJECT_VOLTS));
            this.intaking = false;
        });
    }

    public Command jiggle() {
        return run(() -> {
            motor.setControl(voltageRequest.withOutput(IntakeConstants.Rollers.JIGGLE_VOLTS));
            this.intaking = false;
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(voltageRequest.withOutput(0));
            this.intaking = false;
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setControl(voltageRequest.withOutput(joystickPosition.getAsDouble() * IntakeConstants.Rollers.VOLTS));
            this.intaking = false;
        });
    }

    private Command autoeject() {
        return Commands.sequence(
                Commands.race(
                        Commands.waitUntil(() -> !intaking), // Immediately cancel if we suddenly stop intaking
                        Commands.sequence(
                                runOnce(() -> motor
                                        .setControl(voltageRequest.withOutput(IntakeConstants.Rollers.EJECT_VOLTS))),
                                Commands.waitTime(IntakeConstants.Rollers.AUTOEJECT_DURATION),
                                runOnce(() -> motor
                                        .setControl(voltageRequest.withOutput(IntakeConstants.Rollers.VOLTS))))),
                runOnce(() -> this.autoejecting = false));
    }

    @Override
    public void periodic() {
        if (AUTOEJECT_ENABLED && intaking && !autoejecting) {
            final var motorCurrent = motor.getTorqueCurrent().getValue();
            if (autoejectTrigger != null) {
                if (!motorCurrent.gte(IntakeConstants.Rollers.AUTOEJECT_CURRENT_THRESH)) {
                    autoejectTrigger = null;
                } else if (autoejectTrigger.hasElapsed(IntakeConstants.Rollers.AUTOEJECT_TIME_THRESH)) {
                    autoejectTrigger = null;
                    autoejecting = true;
                    CommandScheduler.getInstance().schedule(autoeject());
                }
            } else {
                if (motorCurrent.gte(IntakeConstants.Rollers.AUTOEJECT_CURRENT_THRESH)) {
                    autoejectTrigger = new Timer();
                    autoejectTrigger.reset();
                }
            }
        }
    }
}
