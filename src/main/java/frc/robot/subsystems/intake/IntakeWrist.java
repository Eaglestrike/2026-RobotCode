package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private final DoubleSubscriber kP = DogLog.tunable("Intake/Wrist/kP", IntakeConstants.Wrist.kP, newkP -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(newkP);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    private final DoubleSubscriber kI = DogLog.tunable("Intake/Wrist/kI", IntakeConstants.Wrist.kI, newkI -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.i(newkI);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    private final DoubleSubscriber kD = DogLog.tunable("Intake/Wrist/kD", IntakeConstants.Wrist.kD, newkD -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.d(newkD);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    // alan can remove the stuff below needed, js for feed forward tuning
    private final DoubleSubscriber kS = DogLog.tunable("Intake/Wrist/kS", IntakeConstants.Wrist.kS, newkS -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kS(newkS);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    private final DoubleSubscriber kV = DogLog.tunable("Intake/Wrist/kV", IntakeConstants.Wrist.kV, newkV -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kV(newkV);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    private final DoubleSubscriber kA = DogLog.tunable("Intake/Wrist/kA", IntakeConstants.Wrist.kA, newkA -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kA(newkA);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    private final DoubleSubscriber kG = DogLog.tunable("Intake/Wrist/kG", IntakeConstants.Wrist.kG, newkG -> {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedForward.kG(newkG);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    });

    // -------------------------------------------------------------------------------------------------

    public IntakeWrist() {
        SparkMaxConfig config = new SparkMaxConfig();

        MAXMotionConfig motion = new MAXMotionConfig()
                .cruiseVelocity(IntakeConstants.Wrist.MAX_VEL_RAD_PER_S)
                .maxAcceleration(IntakeConstants.Wrist.MAX_ACC_RAD_PER_S2);

        FeedForwardConfig ff = new FeedForwardConfig()
                .kS(IntakeConstants.Wrist.kS)
                .kV(IntakeConstants.Wrist.kV)
                .kA(IntakeConstants.Wrist.kA)
                .kCosRatio(IntakeConstants.Wrist.GEAR_RATIO)
                .kCos(IntakeConstants.Wrist.kG);

        config.closedLoop.pid(kP.get(), kI.get(), kD.get());
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.apply(motion);
        config.closedLoop.apply(ff);
        config.smartCurrentLimit(IntakeConstants.Wrist.STALL_LIMIT);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // -------------------------------------------------------------------------------------------------

    public Command moveTo(double position) {
        return runOnce(() -> {
            controller.setSetpoint(position, ControlType.kMAXMotionPositionControl);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.getEncoder().setPosition(IntakeConstants.Wrist.OFFSET);
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

}
