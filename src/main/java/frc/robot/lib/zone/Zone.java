package frc.robot.lib.zone;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;

public class Zone {
    protected final double xMin, xMax, yMin, yMax;

    public Zone(double xMin, double xMax, double yMin, double yMax) {
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
    }

    public Zone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
        this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    // current zone
    public Trigger contains(Supplier<Pose2d> poseSupplier) {
        return new Trigger(() -> this.containsPoint(poseSupplier.get().getTranslation()));
    }

    protected boolean containsPoint(Translation2d point) {
        return point.getX() >= xMin && point.getX() <= xMax && point.getY() >= yMin && point.getY() <= yMax;
    }

    // predictive zone
    public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
        return new Trigger(() -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
    }

    protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
        return (point.getY() >= yMin && point.getY() <= yMax)
                && ((point.getX() >= xMin && point.getX() <= xMax)
                        || (point.getX() < xMin
                                && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) >= xMin - point.getX())
                        || (point.getX() > xMax
                                && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) <= xMax - point.getX()));
    }

    // utility methods
    public Zone mirroredX() {
        return new Zone(
                FieldConstants.fieldLength - xMax,
                FieldConstants.fieldLength - xMin,
                yMin,
                yMax);
    }

    public Zone mirroredY() {
        return new Zone(
                xMin,
                xMax,
                FieldConstants.fieldWidth - yMax,
                FieldConstants.fieldWidth - yMin);
    }

    // list of corners, with the bottom left corner repeated at the end to form a
    // closed loop
    public Translation2d[] getCorners() {
        return new Translation2d[] {
                new Translation2d(xMin, yMin),
                new Translation2d(xMax, yMin),
                new Translation2d(xMax, yMax),
                new Translation2d(xMin, yMax),
                new Translation2d(xMin, yMin)
        };
    }
}