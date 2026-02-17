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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;

/**
 * Flywheel subsystem using shared NT table for bidirectional communication.
 * Pattern from JerryRoboticsArm2: robot periodic() reads commands and writes status
 * to the SAME NT table that Streamlit reads/writes.
 */
public class ShooterFlywheels extends SubsystemBase {
    private final SparkFlex motor;
    private final SparkClosedLoopController closedLoopController;
    private SparkFlexConfig config = new SparkFlexConfig();
    private double lastTargetRPM = 0.0;

    /** For debugging: get the current target RPM */
    public double getTargetRPM() { return lastTargetRPM; }
    private final int motorId;

    /** Shared NT table — Streamlit 和 robot 共用 */
    private final NetworkTable nt;
    private double prevNtTargetRPM = Double.NaN;
    /** 当 NT 手动控制模式开启时，外部 setVelocity() 调用被忽略 */
    private boolean ntOverride = false;

    public ShooterFlywheels() {
        nt = NetworkTableInstance.getDefault().getTable("Shooter").getSubTable("Flywheel");

        // Publish defaults so Streamlit can see them immediately
        nt.getEntry("TargetRPM").setDouble(0.0);
        nt.getEntry("kP").setDouble(FlywheelConstants.kP);
        nt.getEntry("kI").setDouble(FlywheelConstants.kI);
        nt.getEntry("kD").setDouble(FlywheelConstants.kD);
        nt.getEntry("maxAccel").setDouble(FlywheelConstants.MAX_ACCEL);
        nt.getEntry("TuneLive").setBoolean(false);
        nt.getEntry("ManualControl").setBoolean(false);
        nt.getEntry("MotorID").setDouble(FlywheelConstants.MOTOR_ID);

        motorId = (int) nt.getEntry("MotorID").getDouble(FlywheelConstants.MOTOR_ID);
        motor = new SparkFlex(motorId, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();

        // Motor configuration
        config.idleMode(IdleMode.kCoast)
                .voltageCompensation(12.0)
                .inverted(true)
                .smartCurrentLimit(FlywheelConstants.CURRENT_LIMIT);

        config.closedLoop
                .p(FlywheelConstants.kP)
                .i(FlywheelConstants.kI)
                .d(FlywheelConstants.kD)
                .outputRange(FlywheelConstants.MIN_OUTPUT, FlywheelConstants.MAX_OUTPUT);

        config.closedLoop.maxMotion
                .maxAcceleration(FlywheelConstants.MAX_ACCEL);

        try {
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        } catch (Exception e) {
            System.err.println("[ShooterFlywheels] motor.configure failed: " + e.getMessage());
        }
    }

    public Command startAtVelocity(double targetRPM) {
        return this.runOnce(() -> setVelocity(targetRPM));
    }

    public void setVelocity(double targetRPM) {
        // 当 ManualControl 开启时，忽略外部 (ShooterController) 的调用
        if (ntOverride) {
            return;
        }
        setVelocityInternal(targetRPM);
    }

    /** 内部方法，不受 ntOverride 影响 */
    private void setVelocityInternal(double targetRPM) {
        lastTargetRPM = targetRPM;
        closedLoopController.setSetpoint(targetRPM, ControlType.kMAXMotionVelocityControl);
    }

    public Command startAtVelocityBasic(double targetRPM) {
        return this.runOnce(() -> {
            closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
        });
    }

    public Command stop() {
        return this.runOnce(() -> motor.stopMotor());
    }

    public double getVelocityRPM() {
        return motor.getEncoder().getVelocity();
    }

    public boolean isAtVelocity(double targetRPM) {
        return Math.abs(getVelocityRPM() - targetRPM) < FlywheelConstants.VELOCITY_TOLERANCE;
    }

    public boolean isAtVelocity(double targetRPM, double toleranceRPM) {
        return Math.abs(getVelocityRPM() - targetRPM) < toleranceRPM;
    }

    @Override
    public void periodic() {
        // ─── 0. 读取 ManualControl 标志 ───
        ntOverride = nt.getEntry("ManualControl").getBoolean(false);

        // ─── 1. 从 NT 读取命令 ───
        double ntRpm = nt.getEntry("TargetRPM").getDouble(Double.NaN);
        
        if (!Double.isNaN(ntRpm) && ntRpm != prevNtTargetRPM) {
            prevNtTargetRPM = ntRpm;
            // 使用 setVelocityInternal 绕过 ntOverride 检查
            setVelocityInternal(ntRpm);
        }

        // Live tuning (PID/maxAccel)
        if (nt.getEntry("TuneLive").getBoolean(false)) {
            config.closedLoop
                    .p(nt.getEntry("kP").getDouble(FlywheelConstants.kP))
                    .i(nt.getEntry("kI").getDouble(FlywheelConstants.kI))
                    .d(nt.getEntry("kD").getDouble(FlywheelConstants.kD));
            config.closedLoop.maxMotion
                    .maxAcceleration(nt.getEntry("maxAccel").getDouble(FlywheelConstants.MAX_ACCEL));
            try {
                motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            } catch (Exception e) { /* ignore */ }
        }

        // ─── 2. 写入状态到同一个 NT 表 ───
        double vel = getVelocityRPM();
        double err = lastTargetRPM - vel;
        boolean atSetpoint = isAtVelocity(lastTargetRPM);

        nt.getEntry("VelocityRPM").setDouble(vel);
        nt.getEntry("CurrentTargetRPM").setDouble(lastTargetRPM);
        nt.getEntry("ErrorRPM").setDouble(err);
        nt.getEntry("AtSetpoint").setBoolean(atSetpoint);
        nt.getEntry("Enabled").setBoolean(lastTargetRPM != 0.0);
        nt.getEntry("AppliedOutput").setDouble(motor.getAppliedOutput());
        nt.getEntry("OutputCurrent").setDouble(motor.getOutputCurrent());
        nt.getEntry("MotorTemp").setDouble(motor.getMotorTemperature());

        // 同时保留 DogLog 文件日志
        DogLog.log("ShooterFlywheels/VelocityRPM", vel);
        DogLog.log("ShooterFlywheels/targetRPM", lastTargetRPM);
        DogLog.log("ShooterFlywheels/ErrorRPM", err);
        DogLog.log("ShooterFlywheels/AtSetpoint", atSetpoint);
    }
}
