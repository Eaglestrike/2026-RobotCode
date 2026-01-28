package frc.robot.subsystems;

import java.util.Map;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterHood;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.serialization.Feeder;
import frc.robot.subsystems.serialization.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheels;

public class Superstructure extends SubsystemBase {
    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    private ShootMode currentMode;
    private boolean isShooting;
    private EagleSwerveDrivetrain drive;

    // private DoubleSubscriber turretang = DogLog.tunable("TurretAng", 0.0);
   

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private IntakeWrist intakeWrist;
    private final Feeder feeder;
    private final Hopper hopper;

// probably should have init mechanism that starts ythe hoppper and flywheels
    public Superstructure(EagleSwerveDrivetrain drive, Climb climb, Shooter shooter, IntakeRollers intakeRollers,
                                        IntakeWrist intakeWrist, Feeder feeder, Hopper hopper) {
            currentMode = ShootMode.SHOOT;
            isShooting = false;
            this.climb = climb;
            this.shooter = shooter;
            this.intakeRollers = intakeRollers;
            this.intakeWrist = intakeWrist;
            this.feeder = feeder;
            this.hopper = hopper;
            this.drive = drive;
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
            climb.moveTo(ClimbConstants.ClimbState.STOWED.getPosition())

        );
    }

    /**
     * Command to climb to a specified level
     * 
     * @param level The level to climb to (REPLACE OBJECT WITH ACTUAL TYPE -
     *              PROBABLY INT OR ENUM)
     * @return SequenceCommand that performs the climbing action
     */
    public Command climb(ClimbConstants.ClimbState level) {
        return Commands.sequence(
            stow(),
            climb.moveTo(level.getPosition())
        );

    }

    public Object getClimbState() {
        return climb;
        
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
        // If shooting, then periodically recalibrate shooter (turret, hood and flywheel
        // speeds)
        // bring shooter flywheel speeds up to speed
        // TODO:
        // if it is facing use the kicker to actually shoot
        // based on mode (ferry or shoot) set different target positions
        if (isShooting){          
            shooter.setFacing(FieldConstants.getAllianceHubPosition().toPose2d());
            ShooterFlywheels.startAtVelocity(ShooterConstants.MAX_VELOCITY);
            boolean facing = shooter.isFacingTarget();     // TODO: add isFacing into shooter and someone else will
                                                       // immplemetn it
            boolean atSpeed = shooter.isAtVelocity(ShooterConstants.MAX_VELOCITY);

            if (shooter.isFacingTarget() && shooter.isAtVelocity(ShooterConstants.MAX_VELOCITY)) {
                shooter.runKicker(true);
            } 
            else {
                shooter.runKicker(false);
            }
        }      
        
        else {
            ShooterFlywheels.startAtVelocity(0.0);
            shooter.runKicker(false);
            shooter.setFacing(FieldConstants.getAllianceHubPosition().toPose2d());
        }

    }
}