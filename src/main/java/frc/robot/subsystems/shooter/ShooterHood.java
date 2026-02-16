package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

/**
 * Hood subsystem using shared NT table for bidirectional communication.
 */
public class ShooterHood extends SubsystemBase {
    private final int motorId;
    private final SparkMax motor;
    private final RelativeEncoder enc;
    private final ProfiledPIDController pid;
    private ArmFeedforward ff;
    private Angle goal = Degrees.of(HoodConstants.MAX_ANGLE); // 默认最陡位置（对应 31.8° 从垂直射击角）
    /** Hood 只有被显式激活后才输出电压，防止 Enable 后堵转发热 */
    private boolean active = false;

    /** Shared NT table */
    private final NetworkTable nt;
    private double prevNtRequestDeg = Double.NaN;

    public ShooterHood() {
        nt = NetworkTableInstance.getDefault().getTable("Shooter").getSubTable("Hood");

        // Publish defaults
        nt.getEntry("RequestDeg").setDouble(HoodConstants.MAX_ANGLE);
        nt.getEntry("Zero").setBoolean(false);
        nt.getEntry("SetPos").setBoolean(false);
        nt.getEntry("SetPosDeg").setDouble(0.0);
        nt.getEntry("TuneLive").setBoolean(false);
        nt.getEntry("MotorID").setDouble(HoodConstants.MOTOR_ID);
        nt.getEntry("kP").setDouble(HoodConstants.kP);
        nt.getEntry("kI").setDouble(HoodConstants.kI);
        nt.getEntry("kD").setDouble(HoodConstants.kD);
        nt.getEntry("kS").setDouble(HoodConstants.kS);
        nt.getEntry("kG").setDouble(HoodConstants.kG);
        nt.getEntry("kV").setDouble(HoodConstants.kV);
        nt.getEntry("kA").setDouble(HoodConstants.kA);
        nt.getEntry("MaxVel").setDouble(HoodConstants.MAX_VEL);
        nt.getEntry("MaxAcc").setDouble(HoodConstants.MAX_ACC);
        nt.getEntry("IZoneDeg").setDouble(HoodConstants.IZONE_DEG);
        nt.getEntry("IMaxVolts").setDouble(HoodConstants.IMAX_VOLTS);

        motorId = (int) nt.getEntry("MotorID").getDouble(HoodConstants.MOTOR_ID);
        motor = new SparkMax(motorId, MotorType.kBrushless);
        enc = motor.getEncoder();

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        cfg.inverted(HoodConstants.MOTOR_INVERTED);
        cfg.encoder.positionConversionFactor(2.0 * Math.PI / HoodConstants.GEAR_RATIO);
        cfg.encoder.velocityConversionFactor(2.0 * Math.PI / HoodConstants.GEAR_RATIO / 60.0);
        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = new ProfiledPIDController(
                HoodConstants.kP, HoodConstants.kI, HoodConstants.kD,
                new TrapezoidProfile.Constraints(HoodConstants.MAX_VEL, HoodConstants.MAX_ACC));
        pid.setTolerance(Math.toRadians(1.0));
        pid.reset(getAngle().magnitude());
        pid.setIntegratorRange(-HoodConstants.IMAX_VOLTS, HoodConstants.IMAX_VOLTS);
        pid.setIZone(Math.toRadians(HoodConstants.IZONE_DEG));

        ff = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV, HoodConstants.kA);

        // ─── SmartDashboard 调参初始值 ───
        SmartDashboard.putNumber("Hood/kP", HoodConstants.kP);
        SmartDashboard.putNumber("Hood/kI", HoodConstants.kI);
        SmartDashboard.putNumber("Hood/kD", HoodConstants.kD);
        SmartDashboard.putNumber("Hood/kS", HoodConstants.kS);
        SmartDashboard.putNumber("Hood/kG", HoodConstants.kG);
        SmartDashboard.putNumber("Hood/kV", HoodConstants.kV);
        SmartDashboard.putNumber("Hood/kA", HoodConstants.kA);
        SmartDashboard.putNumber("Hood/MaxVel", HoodConstants.MAX_VEL);
        SmartDashboard.putNumber("Hood/MaxAcc", HoodConstants.MAX_ACC);
        SmartDashboard.putBoolean("Hood/Zero", false);
        SmartDashboard.putNumber("Hood/SetGoalDeg", goal.in(Degrees));
    }

    public Angle getAngle() {
        return Radians.of(enc.getPosition());
    }

    public Command goToPos(Angle position) {
        return runOnce(() -> setGoal(position));
    }

    public void setGoal(Angle position) {
        goal = position;
        if (!active) {
            active = true;
            pid.reset(getAngle().in(Radians)); // 激活时从当前位置开始，避免突然跳跃
        }
    }

    /**
     * 增量调整 Hood 目标角度（自动 clamp 到有效范围）
     */
    public Command adjustGoal(double deltaDeg) {
        return runOnce(() -> {
            if (!active) {
                // 首次激活：从当前编码器位置开始，而不是默认 goal
                double startDeg = MathUtil.clamp(getAngle().in(Degrees),
                        HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
                goal = Degrees.of(startDeg);
                active = true;
                pid.reset(getAngle().in(Radians));
            }
            double currentGoalDeg = goal.in(Degrees);
            double newGoalDeg = MathUtil.clamp(
                    currentGoalDeg + deltaDeg,
                    HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
            goal = Degrees.of(newGoalDeg);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            active = false;
            motor.setVoltage(0);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            System.out.println("Zeroing Shooter Hood");
            enc.setPosition(HoodConstants.ENCODER_OFFSET);
        });
    }

    public Command setManualVoltage(double joystickPosition) {
        return runOnce(() -> motor.setVoltage(joystickPosition * HoodConstants.MAX_VOLTS));
    }

    @Override
    public void periodic() {
        if (!DriverStation.isEnabled() || !active) {
            motor.setVoltage(0.0);
            SmartDashboard.putBoolean("Hood/Active", active);
            SmartDashboard.putNumber("Hood/CurrAngDeg", getAngle().in(Degrees));
            SmartDashboard.putNumber("Hood/OutputVolts", 0.0);
            return;
        }

        // ─── 1. 从 SmartDashboard / NT 读取命令 ───
        // SmartDashboard Zero 按钮
        if (SmartDashboard.getBoolean("Hood/Zero", false)) {
            enc.setPosition(HoodConstants.ENCODER_OFFSET);
            SmartDashboard.putBoolean("Hood/Zero", false);
            nt.getEntry("Zero").setBoolean(false);
        }
        // NT Zero 按钮 (兼容 Streamlit)
        if (nt.getEntry("Zero").getBoolean(false)) {
            enc.setPosition(HoodConstants.ENCODER_OFFSET);
            nt.getEntry("Zero").setBoolean(false);
        }

        // Set position (calibration, NT only)
        if (nt.getEntry("SetPos").getBoolean(false)) {
            double setDeg = nt.getEntry("SetPosDeg").getDouble(0.0);
            enc.setPosition(Math.toRadians(setDeg));
            nt.getEntry("SetPos").setBoolean(false);
        }

        // SmartDashboard 目标角度设置
        double sdGoalDeg = SmartDashboard.getNumber("Hood/SetGoalDeg", Double.NaN);
        if (!Double.isNaN(sdGoalDeg) && Math.abs(sdGoalDeg - goal.in(Degrees)) > 0.1) {
            setGoal(Degrees.of(sdGoalDeg));
        }

        // NT Request angle (兼容 Streamlit / 监控脚本)
        double ntReqDeg = nt.getEntry("RequestDeg").getDouble(Double.NaN);
        if (!Double.isNaN(ntReqDeg) && ntReqDeg != prevNtRequestDeg) {
            prevNtRequestDeg = ntReqDeg;
            setGoal(Degrees.of(ntReqDeg));
        }

        // ─── SmartDashboard 实时调参（每周期读取） ───
        pid.setP(SmartDashboard.getNumber("Hood/kP", HoodConstants.kP));
        pid.setI(SmartDashboard.getNumber("Hood/kI", HoodConstants.kI));
        pid.setD(SmartDashboard.getNumber("Hood/kD", HoodConstants.kD));
        pid.setConstraints(new TrapezoidProfile.Constraints(
                SmartDashboard.getNumber("Hood/MaxVel", HoodConstants.MAX_VEL),
                SmartDashboard.getNumber("Hood/MaxAcc", HoodConstants.MAX_ACC)));
        ff = new ArmFeedforward(
                SmartDashboard.getNumber("Hood/kS", HoodConstants.kS),
                SmartDashboard.getNumber("Hood/kG", HoodConstants.kG),
                SmartDashboard.getNumber("Hood/kV", HoodConstants.kV),
                SmartDashboard.getNumber("Hood/kA", HoodConstants.kA));

        // NT Live tuning (兼容 Streamlit)
        if (nt.getEntry("TuneLive").getBoolean(false)) {
            pid.setP(nt.getEntry("kP").getDouble(HoodConstants.kP));
            pid.setI(nt.getEntry("kI").getDouble(HoodConstants.kI));
            pid.setD(nt.getEntry("kD").getDouble(HoodConstants.kD));
            pid.setConstraints(new TrapezoidProfile.Constraints(
                    nt.getEntry("MaxVel").getDouble(HoodConstants.MAX_VEL),
                    nt.getEntry("MaxAcc").getDouble(HoodConstants.MAX_ACC)));
            pid.setIZone(Math.toRadians(nt.getEntry("IZoneDeg").getDouble(HoodConstants.IZONE_DEG)));
            pid.setIntegratorRange(
                    -nt.getEntry("IMaxVolts").getDouble(HoodConstants.IMAX_VOLTS),
                    nt.getEntry("IMaxVolts").getDouble(HoodConstants.IMAX_VOLTS));
            ff = new ArmFeedforward(
                    nt.getEntry("kS").getDouble(HoodConstants.kS),
                    nt.getEntry("kG").getDouble(HoodConstants.kG),
                    nt.getEntry("kV").getDouble(HoodConstants.kV),
                    nt.getEntry("kA").getDouble(HoodConstants.kA));
        }

        // ─── 2. 控制 + 写入状态 ───
        goal = Radians.of(MathUtil.clamp(goal.in(Radians),
                Math.toRadians(HoodConstants.MIN_ANGLE), Math.toRadians(HoodConstants.MAX_ANGLE)));
        pid.setGoal(goal.in(Radians));

        double curDeg = getAngle().in(Degrees);
        double goalDeg = goal.in(Degrees);
        double errDeg = goalDeg - curDeg;
        double setpointVel = pid.getSetpoint().velocity;

        double pidVolts = pid.calculate(getAngle().in(Radians));
        double ffVolts = ff.calculate(getAngle().in(Radians), setpointVel);
        double volts = pidVolts + ffVolts;
        double outputVolts = MathUtil.clamp(volts, -HoodConstants.MAX_VOLTS, HoodConstants.MAX_VOLTS);
        motor.setVoltage(outputVolts);

        // Write status to shared NT table
        nt.getEntry("CurrAngDeg").setDouble(curDeg);
        nt.getEntry("GoalDeg").setDouble(goalDeg);
        nt.getEntry("GoalRad").setDouble(goal.magnitude());
        nt.getEntry("ErrorDeg").setDouble(errDeg);
        nt.getEntry("VelocityDegPerS").setDouble(Math.toDegrees(enc.getVelocity()));
        nt.getEntry("PIDVolts").setDouble(pidVolts);
        nt.getEntry("FFVolts").setDouble(ffVolts);
        nt.getEntry("OutputVolts").setDouble(outputVolts);
        nt.getEntry("AtTarget").setBoolean(pid.atGoal());

        // SmartDashboard 状态显示
        SmartDashboard.putNumber("Hood/CurrAngDeg", curDeg);
        SmartDashboard.putNumber("Hood/GoalDeg", goalDeg);
        SmartDashboard.putNumber("Hood/ErrorDeg", errDeg);
        SmartDashboard.putNumber("Hood/OutputVolts", outputVolts);
        SmartDashboard.putNumber("Hood/PIDVolts", pidVolts);
        SmartDashboard.putNumber("Hood/FFVolts", ffVolts);
        SmartDashboard.putNumber("Hood/VelDegPerS", Math.toDegrees(enc.getVelocity()));
        SmartDashboard.putBoolean("Hood/AtTarget", pid.atGoal());
        SmartDashboard.putBoolean("Hood/Active", active);

        // DogLog 文件日志
        DogLog.log("ShooterHood/CurrAngDeg", curDeg);
        DogLog.log("ShooterHood/GoalDeg", goalDeg);
        DogLog.log("ShooterHood/OutputVolts", outputVolts);
    }

    public Pose3d getComponentPose3d(Pose3d turretPose) {
        double offsetX = 0.106;
        double offsetY = 0.0;
        var yaw = turretPose.getRotation().getZ();
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
