package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
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
        return Commands.none();
    }

    public void setFacing(Pose2d targetPose) {
        turret.setFacing(targetPose);
        // TODO: Calculate hood angle based on distance to targetPose
        hood.goToPos(Angle.ofBaseUnits(0, Radians));

        DogLog.log("Sim/ComponentPoses", new Pose3d[] {
            new Pose3d(0, 0, 0, new Rotation3d()),
            this.turret.getComponentPose3d(),
            this.hood.getComponentPose3d(this.turret.getComponentPose3d()),
        });
    }

    public boolean isAtVelocity() {
        // TODO Auto-generated method stub
        return true;
    }

    public boolean isFacingTarget() {
        // TODO Auto-generated method stub
        return true;
    }

    

    public ShooterFlywheels getFlywheels() {
        return flywheel;
    }
}
