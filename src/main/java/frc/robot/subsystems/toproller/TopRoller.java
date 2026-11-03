
package frc.robot.subsystems.toproller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class TopRoller extends SubsystemBase {
    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    enum RollerState {
        RUNNING,
        MANUAL,
        IDLE
    }

    private RollerState currState = RollerState.IDLE;
    private double manualVolts = 0;

    public TopRoller() {
        motor = new TalonFX(TopRollerConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);

        var cfg = motor.getConfigurator();
        var currentCfg = new CurrentLimitsConfigs();

        currentCfg.StatorCurrentLimit = TopRollerConstants.CURRENT_LIMIT;
        currentCfg.StatorCurrentLimitEnable = true;

        cfg.apply(currentCfg);
        motor.optimizeBusUtilization();
    }

    public void start() {
        currState = RollerState.RUNNING;
    }

    public void stop() {
        currState = RollerState.IDLE;
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            manualVolts = joystickPosition.getAsDouble() * TopRollerConstants.RUN_VOLTS;
            currState = RollerState.MANUAL;
        });
    }

    @Override
    public void periodic() {
        switch (currState) {
            case RUNNING:
                motor.setControl(voltageRequest.withOutput(TopRollerConstants.RUN_VOLTS));
                break;

            case MANUAL:
                motor.setControl(voltageRequest.withOutput(manualVolts));
                break;

            case IDLE:
                motor.setControl(voltageRequest.withOutput(0));
                break;
        }

        DogLog.forceNt.log("TopRoller/CurrentState", currState.toString());
        DogLog.forceNt.log("TopRoller/ManualVoltage", manualVolts);
    }
}
