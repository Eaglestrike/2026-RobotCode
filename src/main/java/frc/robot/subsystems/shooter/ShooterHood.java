package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;

public class ShooterHood extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration cfg = new TalonFXConfiguration();
    private final PositionVoltage request = new PositionVoltage(0);
    private Angle targetAngle = Degrees.of(HoodConstants.DEFAULT_GOAL);
    private boolean zeroed = false;

    public ShooterHood() {
        this.motor = new TalonFX(HoodConstants.MOTOR_ID, RobotContainer.MECHANISM_CANBUS);
        cfg.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        cfg.Slot0.kP = HoodConstants.kP;
        cfg.Slot0.kD = HoodConstants.kD;
        cfg.Slot0.kS = HoodConstants.kS;
        cfg.Slot0.kV = HoodConstants.kV;
        cfg.Slot0.kA = HoodConstants.kA;
        cfg.Slot0.kG = HoodConstants.kG;

        cfg.Voltage.PeakForwardVoltage = HoodConstants.MAX_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -HoodConstants.MAX_VOLTS;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.MAX_ANGLE / 360.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.MIN_ANGLE / 360.0;

        this.motor.getConfigurator().apply(cfg);

        var talonFXSim = motor.getSimState();
        talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

        motor.setPosition(Degrees.of(HoodConstants.OFFSET));

        DogLog.tunable("ShooterHood/Request Degrees", HoodConstants.DEFAULT_GOAL, x -> {
            CommandScheduler.getInstance().schedule(goToPos(Degrees.of(x)));
        });

        // DogLog.log("ShooterHood/isZeroed", zeroed);
    }

    /**
     * @return Current shooting angle
     */
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    public Command goToPos(Angle shootingAngle) {
        return runOnce(() -> setGoal(shootingAngle));
    }

    /**
     * Set goal in shooting angle (degrees from horizontal).
     * 75.11° = most horizontal (hard stop), 26° = most tilted
     */
    public void setGoal(Angle shootingAngle) {
        if (!zeroed)
            return;
        targetAngle = shootingAngle;
        motor.setControl(request.withPosition(shootingAngle));
    }

    public Command setManualVoltage(DoubleSupplier joystickPosition) {
        return startRun(
                () -> {
                    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
                    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
                    motor.getConfigurator().apply(cfg);
                },
                () -> {
                    motor.setVoltage(joystickPosition.getAsDouble() * HoodConstants.MAX_VOLTS);
                }).finallyDo(() -> {
                    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
                    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
                    motor.getConfigurator().apply(cfg);
                });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setVoltage(0);
        });
    }

    public Command zero() {
        return runOnce(() -> {
            System.out.println("Zeroing Shooter Hood");
            motor.setPosition(Degrees.of(HoodConstants.OFFSET));
            zeroed = true;
            // DogLog.log("ShooterHood/isZeroed", zeroed);
        });
    }

    @Override
    public void periodic() {
        // DogLog.log("ShooterHood/ShootingAngle", getAngle().in(Degrees), "Degrees");
        // DogLog.log("ShooterHood/AtTarget", atTarget());
    }

    public boolean atTarget() {
        return getAngle().isNear(targetAngle, HoodConstants.ANGLE_TOLERANCE);
    }

    /* sim */
    private static final double gearRatio = HoodConstants.GEAR_RATIO;
    private final DCMotor gearbox = DCMotor.getKrakenX44(1);
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, 0.001, gearRatio),
            gearbox);

    @Override
    public void simulationPeriodic() {
        var talonFXSim = motor.getSimState();
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        sim.setInputVoltage(motorVoltage.in(Volts));
        sim.update(0.020);

        talonFXSim.setRawRotorPosition(sim.getAngularPosition().times(gearRatio));
        talonFXSim.setRotorVelocity(sim.getAngularVelocity().times(gearRatio));
        // DogLog.log("ShooterHood/SimPosition",
        // sim.getAngularPosition().times(gearRatio).in(Degrees), "Degrees");
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