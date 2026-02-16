package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Shooter extends SubsystemBase {
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheels;
    private final ShooterController controller;
    private boolean manualTestMode = false;

    public Shooter(EagleSwerveDrivetrain drive) {
        turret = new ShooterTurret();
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();
        controller = new ShooterController(drive, turret, hood, flywheels);
    }

    /**
     * Get the unified shooter controller for aiming
     */
    public ShooterController getController() {
        return controller;
    }

    /**
     * Set target and enable aiming
     */
    public void aimAt(Pose3d target) {
        controller.setTarget(target);
        controller.setEnabled(true);
    }

    /**
     * Disable aiming (go to idle)
     */
    public void stopAiming() {
        controller.setEnabled(false);
    }

    /**
     * Check if ready to shoot
     */
    public boolean isReadyToShoot() {
        return controller.isReadyToShoot();
    }

    public Command stowShooter() {
        return Commands.parallel(
            turret.requestPosition(Degrees.of(0)),
            hood.goToPos(Degrees.of(ShooterConstants.HoodConstants.MIN_ANGLE)),
            flywheels.stop()
        );
    }

    /** Enable/disable manual test mode (prevents Superstructure from overriding) */
    public void setManualTestMode(boolean manual) {
        this.manualTestMode = manual;
        if (manual) {
            controller.setEnabled(false);
        }
    }

    public boolean isManualTestMode() {
        return manualTestMode;
    }

    public ShooterFlywheels getFlywheels() {
        return flywheels;
    }

    public ShooterTurret getTurret() {
        return turret;
    }

    public ShooterHood getHood() {
        return hood;
    }

    @Override
    public void periodic() {
        // Update the controller every loop
        controller.update();
    }
}
