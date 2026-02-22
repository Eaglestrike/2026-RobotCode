package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;

public class Shooter extends SubsystemBase {
    private final ShooterTurret turret;
    private final ShooterHood hood;
    private final ShooterFlywheels flywheel;

    public Shooter(EagleSwerveDrivetrain drive) {
        turret = new ShooterTurret(drive);
        hood = new ShooterHood();
        flywheel = new ShooterFlywheels();
    }

    public Command stowShooter() {
        return Commands.parallel(
                turret.requestPosition(Rotations.of(ShooterConstants.TurretConstants.STOW_POSITION)),
                hood.requestPosition(Radians.of(ShooterConstants.HoodConstants.STOW_ANGLE)),
                flywheel.startAtVelocity(ShooterConstants.FlywheelConstants.IDLE_VELOCITY));
    }

    public void setFacing(Pose3d targetPose) {
        turret.setFacing(targetPose.toPose2d());
        // TODO: Calculate hood angle based on distance to targetPose - this cna either
        // be done in Shooter or create a ShooterUtils class with all the math
        double hoodAngle = 0;

        // TODO: Calculate this in meters per second based on distance to targetPose and
        // desired launch angle
        double targetBallVel = 0;

        // CommandScheduler.getInstance().schedule(Commands.parallel(
        // hood.goToPos(Radians.of(hoodAngle)),
        // flywheel.startAtVelocity(ShooterConstants.BALL_VEL_TO_FLYWHEEL_VEL.get(targetBallVel))));
    }

    public boolean isReady() {
        return true; // testing purposes only
        // return isAtVelocity() && isFacingTarget();
    }

    private boolean isAtVelocity() {
        return flywheel.atVelocity();
    }

    private boolean isFacingTarget() {
        return turret.atTarget() && hood.atTarget();
    }

    public ShooterFlywheels getFlywheels() {
        return flywheel;
    }

    public ShooterTurret getTurret() {
        return turret;
    }

    public ShooterHood getHood() {
        return hood;
    }
}
