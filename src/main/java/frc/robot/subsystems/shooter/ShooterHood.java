package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

public class ShooterHood extends SubsystemBase {
    private TalonFX motor;
    private final PositionVoltage request = new PositionVoltage(HoodConstants.BACKWARD_LIMIT);
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private Angle targetAngle = Rotations.of(0);

    public ShooterHood() {
        motor = new TalonFX(HoodConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        cfg.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        cfg.Slot0.kP = HoodConstants.kP;
        cfg.Slot0.kD = HoodConstants.kD;
        cfg.Slot0.kS = HoodConstants.kS;
        cfg.Slot0.kV = HoodConstants.kV;
        cfg.Slot0.kA = HoodConstants.kA;
        cfg.Slot0.kG = HoodConstants.kG;

        cfg.Voltage.PeakForwardVoltage = HoodConstants.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -HoodConstants.MAX_VOLTS;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.FORWARD_LIMIT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.BACKWARD_LIMIT;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        this.motor.getConfigurator().apply(cfg);

        var talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        DogLog.tunable("ShooterHood/Request Rotations", 0.0, x -> {
            CommandScheduler.getInstance().schedule(requestPosition(Rotations.of(x)));
        });
    }

    public Command requestPosition(Angle position) {
        return runOnce(() -> {
            targetAngle = position;
            motor.setControl(request.withPosition(position));
        });
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return run(() -> {
            motor.setVoltage(joystickPosition.getAsDouble() * HoodConstants.MAX_VOLTS);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            this.motor.stopMotor();
        });
    }

    public Command zero() {
        return runOnce(() -> {
            motor.setPosition(HoodConstants.OFFSET);
        });
    }

    public boolean atTarget() {
        return motor.getPosition().getValue().isNear(targetAngle, HoodConstants.ANGLE_TOLERANCE);
    }

    @Override
    public void periodic() {
        DogLog.log("ShooterHood/Position", motor.getPosition().getValue().in(Degrees), "Degrees");
        DogLog.log("ShooterHood/AtTarget", atTarget() ? 1 : 0, "Boolean");
    }

    /* sim */
    private static final double gearRatio = HoodConstants.GEAR_RATIO;
    private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                    gearbox,
                    0.022,
                    gearRatio),
            gearbox,
            gearRatio,
            0.252559,
            -0.22,
            0,
            true,
            -0.22);

    @Override
    public void simulationPeriodic() {
        var talonFXSim = motor.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();
        DogLog.log("ShooterHood/MotorVoltage", motorVoltage.in(Volts), "Volts");

        sim.setInputVoltage(motorVoltage.in(Volts));
        sim.update(0.020);

        talonFXSim.setRawRotorPosition(sim.getAngleRads() * gearRatio);
        talonFXSim.setRotorVelocity(sim.getVelocityRadPerSec() * gearRatio);
        DogLog.log("ShooterHood/SimPosition", sim.getAngleRads(), "Radians");
    }

    public Pose3d getComponentPose3d(Pose3d turretPose) {
        // offset of the hood relative to the turret frame (forward, lateral, up)
        double offsetX = 0.106;
        double offsetY = 0.0;

        var yaw = turretPose.getRotation().getZ();

        // rotate the XY offset by the turret yaw to transform into field coordinates
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double fieldOffsetX = cos * offsetX - sin * offsetY;
        double fieldOffsetY = sin * offsetX + cos * offsetY;

        return new Pose3d(
                turretPose.getX() + fieldOffsetX,
                turretPose.getY() + fieldOffsetY,
                turretPose.getZ() + 0.0713,
                new Rotation3d(0, this.motor.getPosition().getValue().in(Radians), yaw));
    }
}