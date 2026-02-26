package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

public class Superstructure extends SubsystemBase {
    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    private ShootMode currentMode;
    public boolean isShooting;

    private final Climb climb;
    private final Shooter shooter;
    private final IntakeRollers intakeRollers;
    private final IntakeWrist intakeWrist;
    private final Feeder feeder;
    private final Hopper hopper;

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

    public Command initialize() {
        return runOnce(() -> shooter.setArmed(false));
    }

    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.sequence(
                intakeRollers.startHopperOut(),
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.MAX_ANGLE)),
                Commands.waitUntil(intakeWrist::clearedHopper),
                intakeRollers.start());

    }

    /**
     * @return runOnce command that schedules the intake to stow
     */
    public Command stowIntake() {
        return Commands.sequence(
                intakeRollers.stop(),
                intakeWrist.requestPosition(Degrees.of(IntakeConstants.Wrist.STOW_ANGLE)));
    }

    public Command stopIntakeRollers() {
        return intakeRollers.stop();
    }

    /**
     * Stows everything, including turret, hood, intake, and climber
     */
    public Command stow() {
        return Commands.parallel(
                intakeRollers.stop(),
                intakeWrist.requestPosition(Degrees.of(
                        IntakeConstants.Wrist.STOW_ANGLE)),
                shooter.stowShooter(),
                runOnce(() -> isShooting = false),
                climb.moveTo(ClimbConstants.ClimbState.STOWED));
    }

    public Command ejectIntake() {
        return Commands.sequence(
                intakeWrist.eject(),
                Commands.waitSeconds(0.5),
                intakeRollers.eject());
    }

    public Command jiggleIntake() {
        return Commands.parallel(
                intakeWrist.jiggle(),
                intakeRollers.jiggle());
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
                climb.moveTo(level));
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
        if (currentMode == ShootMode.FERRY) {
            shooter.setFacing(FieldConstants.getAllianceOutpostPosition());
        } else {
            shooter.setFacing(FieldConstants.getAllianceHubPosition());
        }

        if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
            isShooting = false;
            shooter.setArmed(false);
            feeder.stopMotor();
            hopper.stopMotor();
            return;
        }

        shooter.setArmed(isShooting);

        if (isShooting) {
            feeder.startMotor();
            hopper.startKicking();
        } else {
            feeder.stopMotor();
            hopper.stopMotor();
        }
    }

    @Override
    public void simulationPeriodic() {
        DogLog.log("Sim/ComponentPoses", new Pose3d[] {
                new Pose3d(0, 0, 0, new Rotation3d()),
                shooter.getTurret().getComponentPose3d(),
                shooter.getHood().getComponentPose3d(shooter.getTurret().getComponentPose3d()),
                new Pose3d(0, 0, 0, new Rotation3d()),
                new Pose3d(0, 0, 0, new Rotation3d()),
        });
    }
}