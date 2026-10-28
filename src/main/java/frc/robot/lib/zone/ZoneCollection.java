package frc.robot.lib.zone;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ZoneCollection {
    protected final Zone[] zones;

    public ZoneCollection(Zone... zones) {
        this.zones = zones;
    }

    public Trigger contains(Supplier<Pose2d> pose) {
        Trigger combined = new Trigger(() -> false);

        for (Zone zone : zones) {
            combined = combined.or(zone.contains(pose));
        }

        return combined;
    }

    public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
        Trigger combined = new Trigger(() -> false);

        for (Zone zone : zones) {
            combined = combined.or(zone.willContain(pose, fieldSpeeds, dt));
        }

        return combined;
    }
}