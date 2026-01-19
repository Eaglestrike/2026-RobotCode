package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.IntakeConstants;

public class IntakeWrist extends SubsystemBase {
    public enum TargetState {
        NONE,
        MOVING,
        AT_TARGET,
        MANUAL
    }

    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.Wrist.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    private TargetState state = TargetState.NONE;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            IntakeConstants.Wrist.MAX_VEL_RAD_PER_S,
            IntakeConstants.Wrist.MAX_ACC_RAD_PER_S2);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0.0, 0.0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0.0, 0.0);

    private double targetRad = 0.0;

    // Custom PID accumulator (mirrors your C++ total error)
    private double totalErr = 0.0;

    private double lastTs = Timer.getFPGATimestamp();

    public IntakeWrist() {
        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.Wrist.INVERTED);
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(40); // TODO if needed

        // Convert motor rotations -> wrist radians
        // motorRotations / gearRatio = mechRotations
        // mechRotations * 2pi = mechRadians
        double posFactorRadPerMotorRev = (2.0 * Math.PI) / IntakeConstants.Wrist.GEAR_RATIO;
        encoder.setPositionConversionFactor(posFactorRadPerMotorRev);
        encoder.setVelocityConversionFactor(posFactorRadPerMotorRev / 60.0); // RPM -> rad/s

        // Apply optional offset at boot (if you use absolute elsewhere, you may
        // overwrite later)
        encoder.setPosition(encoder.getPosition() - IntakeConstants.Wrist.ENCODER_OFFSET_RAD);

        // Initialize states from current measurement
        double curr = getPositionRad();
        goal = new TrapezoidProfile.State(curr, 0.0);
        setpoint = new TrapezoidProfile.State(curr, 0.0);
        targetRad = curr;
    }

    public TargetState getState() {
        return state;
    }

    public double getTargetRad() {
        return targetRad;
    }

    public double getPositionRad() {
        return encoder.getPosition();
    }

    public double getVelocityRadPerSec() {
        return encoder.getVelocity();
    }

    public void moveToRad(double radians) {
        state = TargetState.MOVING;

        targetRad = MathUtil.clamp(
                radians,
                IntakeConstants.Wrist.MIN_ANGLE_RAD,
                IntakeConstants.Wrist.MAX_ANGLE_RAD);

        goal = new TrapezoidProfile.State(targetRad, 0.0);

        resetFFAndI();
    }

    public void setState(TargetState s) {
        state = s;
    }

    public void zeroHere() {
        encoder.setPosition(0.0);
        goal = new TrapezoidProfile.State(0.0, 0.0);
        setpoint = new TrapezoidProfile.State(0.0, 0.0);
        targetRad = 0.0;
        resetFFAndI();
    }

    public void manualVoltage(double joystick) {
        state = TargetState.MANUAL;
        double volts = IntakeConstants.Wrist.MAX_VOLTS * joystick;
        motor.setVoltage(MathUtil.clamp(volts, -IntakeConstants.Wrist.MAX_VOLTS, IntakeConstants.Wrist.MAX_VOLTS));
    }

    public boolean profileDone() {
        // Equivalent to your m_trap.isFinished(); but with WPILib profile:
        // We’ll treat "done" as: near goal position AND near goal velocity.
        return Math.abs(goal.position - getPositionRad()) < IntakeConstants.Wrist.AT_TARGET_TOL_RAD
                && Math.abs(getVelocityRadPerSec()) < Math.toRadians(5.0);
    }

    private void resetFFAndI() {
        totalErr = 0.0;

        // Regenerate: start setpoint from current measurement (like your
        // regenerate(m_currPose))
        setpoint = new TrapezoidProfile.State(getPositionRad(), getVelocityRadPerSec());
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTs;
        lastTs = now;

        // Guard dt
        if (dt <= 0.0 || dt > 0.1)
            dt = 0.02;

        if (state == TargetState.MANUAL)
            return;

        double voltsCmd;
        switch (state) {
            case AT_TARGET:
            case MOVING:
                voltsCmd = calculateVoltage(dt);
                if (Math.abs(getPositionRad() - targetRad) < IntakeConstants.Wrist.AT_TARGET_TOL_RAD) {
                    state = TargetState.AT_TARGET;
                }
                break;

            case NONE:
            default:
                voltsCmd = 0.0;
                break;
        }

        // Safety hard stop
        double pos = getPositionRad();
        if (pos > IntakeConstants.Wrist.MAX_ANGLE_RAD || pos < IntakeConstants.Wrist.MIN_ANGLE_RAD) {
            voltsCmd = 0.0;
        }

        voltsCmd = MathUtil.clamp(voltsCmd, -IntakeConstants.Wrist.MAX_VOLTS, IntakeConstants.Wrist.MAX_VOLTS);
        motor.setVoltage(voltsCmd);
    }

    private double calculateVoltage(double dt) {
        // Step profile forward one loop like your m_trap.currentPose()
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        setpoint = profile.calculate(dt);

        double currPos = getPositionRad();
        double currVel = getVelocityRadPerSec();

        double posErr = setpoint.position - currPos;
        double velErr = setpoint.velocity - currVel;

        if (Math.abs(posErr) < IntakeConstants.Wrist.I_ZONE_RAD) {
            totalErr += posErr * dt;
        } else {
            totalErr = 0.0;
        }
        totalErr = MathUtil.clamp(totalErr, -IntakeConstants.Wrist.I_MAX / Math.max(IntakeConstants.Wrist.kI, 1e-9),
                IntakeConstants.Wrist.I_MAX / Math.max(IntakeConstants.Wrist.kI, 1e-9));

        // pid = kP*posErr + kD*velErr + kI*totalErr
        double pid = IntakeConstants.Wrist.kP * posErr
                + IntakeConstants.Wrist.kD * velErr
                + IntakeConstants.Wrist.kI * totalErr;

        // Target acceleration: derived from profile step
        // (profile doesn't directly provide accel; approximate via dv/dt)
        // NOTE: This is similar to your finite diff acc.
        double targetAcc = (setpoint.velocity - currVel) / dt;

        // ff = kG * pow(|cos(currPos)|, kGPower) * sign(cos(currPos)) + kA*acc + kV*vel
        // + kS*sign(vel)
        double cos = Math.cos(currPos);
        double grav = IntakeConstants.Wrist.kG
                * Math.pow(Math.abs(cos), IntakeConstants.Wrist.kGPower)
                * Math.signum(cos);

        double signVel = Math.signum(setpoint.velocity); // your C++ used target vel sign
        double ff = grav
                + IntakeConstants.Wrist.kA * targetAcc
                + IntakeConstants.Wrist.kV * setpoint.velocity
                + IntakeConstants.Wrist.kS * signVel;

        return pid + ff;
    }
}
