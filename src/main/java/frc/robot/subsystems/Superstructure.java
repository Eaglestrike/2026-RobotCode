package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.serialization.Feeder;
import frc.robot.subsystems.serialization.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Superstructure extends SubsystemBase {
    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    private ShootMode currentMode;
    private boolean isShooting;

    // private DoubleSubscriber turretang = DogLog.tunable("TurretAng", 0.0);

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private IntakeWrist intakeWrist;
    private final Feeder feeder;
    private final Hopper hopper;

    
    // probably should have init mechanism that starts ythe hoppper and flywheels
    public Superstructure(Climb climb, Shooter shooter, IntakeRollers intakeRollers,
            IntakeWrist intakeWrist, Feeder feeder, Hopper hopper) {
        currentMode = ShootMode.SHOOT;
        isShooting = false;
        this.climb = climb;
        this.shooter = shooter;
        this.intakeRollers = intakeRollers;
        this.intakeWrist = intakeWrist;
        this.feeder = feeder;
        this.hopper = hopper;
    }

    public void initialize() {
        // TODO Auto-generated method stub
        hopper.startMotor();
        shooter.getFlywheels().startAtVelocity(ShooterConstants.IDLE_VELOCITY);
    }
    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.sequence(
                intakeRollers.start(),
                intakeWrist.moveTo(IntakeConstants.Wrist.MAX_ANGLE_RAD)
        );
    }

    /**
     * @return runOnce command that schedules the intake to stow
     */
    public Command stowIntake() {
        return Commands.sequence(
                intakeRollers.stop(),
                intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION)
            );
    }

    /**
     * Stows everything, including turret, hood, intake, and climber
     */
    public Command stow() {
        return Commands.parallel(
                intakeRollers.stop(),
                intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION),
                shooter.stowShooter(),
                climb.moveTo(ClimbConstants.ClimbState.STOWED)
        );
    }

    /**
     * Command to climb to a specified level
     * 
     * @param level The level to climb to
     * @return SequenceCommand that performs the climbing action
     */
    public Command climb(ClimbConstants.ClimbState level) {
        return Commands.sequence(
                stow(),
                climb.moveTo(level)
        );
    }

    public ClimbConstants.ClimbState getClimbState() {
        return climb.getState();
    }

    public void setMode(ShootMode mode) {
        currentMode = mode;
    }

    /**
     * Turns shooting on or off
     */
    public void setShooting(boolean shooting) {
        isShooting = shooting;
    }

    @Override
    public void periodic() {
        if(isShooting) {
            if (currentMode == ShootMode.FERRY) {
                shooter.setFacing(FieldConstants.getAllianceOutpostPosition().toPose2d());
            } else if (currentMode == ShootMode.SHOOT) {
                shooter.setFacing(FieldConstants.getAllianceHubPosition().toPose2d());
            }
            shooter.setFacing(FieldConstants.getAllianceHubPosition().toPose2d());
            //TODO: set flywheel speeds based on distance to target
            shooter.getFlywheels().startAtVelocity(ShooterConstants.MAX_VELOCITY);

            if (shooter.isFacingTarget() && shooter.isAtVelocity()) {
                feeder.startMotor();
            } else {
                feeder.stopMotor();
            }
        }
        else {
            shooter.getFlywheels().startAtVelocity(ShooterConstants.IDLE_VELOCITY);
            feeder.stopMotor();
            shooter.setFacing(FieldConstants.getAllianceHubPosition().toPose2d());
        }
    }
}