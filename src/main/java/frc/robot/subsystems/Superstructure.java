package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterHood;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.serialization.Feeder;
import frc.robot.subsystems.serialization.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure extends SubsystemBase {
    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    private ShootMode currentMode;
    private boolean isShooting;
    private EagleSwerveDrivetrain drive;

    // private DoubleSubscriber turretang = DogLog.tunable("TurretAng", 0.0);
    private ShooterTurret turret;
    private ShooterHood hood;

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private IntakeWrist intakeWrist;
    private final Feeder feeder;
    private final Hopper hopper;


                                /**
                                 * TODO: Modify the constructor to initialize using all the other subsystems
                                 */
        public Superstructure(EagleSwerveDrivetrain driveClimb climb, Shooter shooter, IntakeRollers intakeRollers,
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
        this.turret = new ShooterTurret(drive);
        this.hood = new ShooterHood();
    }

    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.sequence(
            intakeRollers.run(IntakeConstants.Rollers.INTAKE_VOLTS),
            intakeWrist.moveTo(IntakeConstants.Wrist.INTAKE_POSITION)
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
            intakeWrist.moveTo(IntakeConstants.Wrist.STOW_POSITION)
            shooter.stowShooter(),
            climb.stowClimb()

        );
    }

    /**
     * Command to climb to a specified level
     * 
     * @param level The level to climb to (REPLACE OBJECT WITH ACTUAL TYPE -
     *              PROBABLY INT OR ENUM)
     * @return SequenceCommand that performs the climbing action
     */
    public Command climb(Object level) {
        return Commands.none();
    }

    public Object getClimbState() {
        return null;
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
        turret.setFacing(FieldConstants.getAllianceHubPosition().toPose2d());
        DogLog.log("Sim/ComponentPoses", new Pose3d[] {
                new Pose3d(0, 0, 0, new Rotation3d()),
                this.turret.getComponentPose3d(),
                this.hood.getComponentPose3d(this.turret.getComponentPose3d()),
        });
    }
}