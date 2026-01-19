package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
  public enum TargetState {
    IN,
    OUT,
    EJECT,
    NONE,
    MANUAL
  }

  private final CANSparkMax side = new CANSparkMax(IntakeConstants.Rollers.SIDE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax top  = new CANSparkMax(IntakeConstants.Rollers.TOP_MOTOR_ID, MotorType.kBrushless);

  private TargetState state = TargetState.NONE;

  public IntakeRollers() {
    side.restoreFactoryDefaults();
    top.restoreFactoryDefaults();

    side.setInverted(IntakeConstants.Rollers.SIDE_INVERTED);
    top.setInverted(IntakeConstants.Rollers.TOP_INVERTED);

    side.setIdleMode(IdleMode.kBrake);
    top.setIdleMode(IdleMode.kBrake);

    side.enableVoltageCompensation(12.0);
    top.enableVoltageCompensation(12.0);

    side.setSmartCurrentLimit(40);
    top.setSmartCurrentLimit(40);
  }

  public void setState(TargetState s) {
    state = s;
  }

  public TargetState getState() {
    return state;
  }

  public void manualVoltage(double joy) {
    state = TargetState.MANUAL;
    side.setVoltage(MathUtil.clamp(joy * IntakeConstants.Rollers.MAX_VOLTS_SIDE,
        -IntakeConstants.Rollers.MAX_VOLTS_SIDE, IntakeConstants.Rollers.MAX_VOLTS_SIDE));
    top.setVoltage(MathUtil.clamp(joy * IntakeConstants.Rollers.MAX_VOLTS_TOP,
        -IntakeConstants.Rollers.MAX_VOLTS_TOP, IntakeConstants.Rollers.MAX_VOLTS_TOP));
  }

  @Override
  public void periodic() {
    if (state == TargetState.MANUAL) return;

    double sideVolts;
    double topVolts;

    switch (state) {
      case IN:
        sideVolts = IntakeConstants.Rollers.IN_VOLTS_SIDE;
        topVolts = IntakeConstants.Rollers.IN_VOLTS_TOP;
        break;
      case OUT:
        sideVolts = IntakeConstants.Rollers.OUT_VOLTS_SIDE;
        topVolts = IntakeConstants.Rollers.OUT_VOLTS_TOP;
        break;
      case EJECT:
        sideVolts = IntakeConstants.Rollers.EJECT_VOLTS_SIDE;
        topVolts = IntakeConstants.Rollers.EJECT_VOLTS_TOP;
        break;
      case NONE:
      default:
        sideVolts = 0.0;
        topVolts = 0.0;
        break;
    }

    sideVolts = MathUtil.clamp(sideVolts, -IntakeConstants.Rollers.MAX_VOLTS_SIDE, IntakeConstants.Rollers.MAX_VOLTS_SIDE);
    topVolts  = MathUtil.clamp(topVolts,  -IntakeConstants.Rollers.MAX_VOLTS_TOP,  IntakeConstants.Rollers.MAX_VOLTS_TOP);

    side.setVoltage(sideVolts);
    top.setVoltage(topVolts);
  }
}
