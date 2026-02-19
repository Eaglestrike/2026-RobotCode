package frc.robot.subsystems.shooter.solver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterTurret;

public class LUTSolver implements ISolver {
    private Config cfg;
    private Translation2d target;
    private ISolver.State state;

    private static final int ITERS = 20;

    public LUTSolver(EagleSwerveDrivetrain drive, ShooterTurret turret) {

    }

    @Override
    public void setTarget(Translation2d target) {
        this.target = target;
    }

    @Override
    public State getState() {
        return this.state;
    }

    @Override
    public void periodic(Pose2d pose, ChassisSpeeds speed) {
        // iteratively calculate aimpoint
        Translation2d aim = target;
        double distance = pose.getTranslation().getDistance(aim);
        for (int i = 0; i < ITERS; i++) {
            double tof = cfg.tofMap.get(distance);

            // update aimpoint to account for robot velocity
            // TODO: translation and rotation velocity should probably use different factors?
            Translation2d disp = pose.exp(speed.toTwist2d(tof)).getTranslation();
            aim = target.minus(disp);
            distance = pose.getTranslation().getDistance(aim);
        }

        double tof = cfg.tofMap.get(distance);
        double error = target.getDistance(aim.plus(pose.exp(speed.toTwist2d(tof)).getTranslation()));

        Rotation2d pitch = cfg.pitchMap.get(distance);
        Rotation2d azimuth = aim.minus(pose.getTranslation()).getAngle().minus(pose.getRotation());
        double rpm = cfg.rpmMap.get(distance);
        this.state = new State(error, pitch, azimuth, rpm);
    }

    public class Config {
        /**
         * Map from shooting distance to hood angle
         */
        public InterpolatingTreeMap<Double, Rotation2d> pitchMap = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), Rotation2d::interpolate);

        /**
         * Map from shooting distance to flywheel RPM
         */
        public InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

        /**
         * Map from shooting distance to time of flight; used
         * to account for the velocity imparted by shooting on the move
         */
        public InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
    }
}
