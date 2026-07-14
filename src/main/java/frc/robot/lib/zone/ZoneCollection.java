package frc.robot.lib.zone;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Time;
import org.wpilib.command2.button.Trigger;

import java.util.function.Supplier;

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

    public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisVelocities> fieldSpeeds, Time dt) {
        Trigger combined = new Trigger(() -> false);

        for (Zone zone : zones) {
            combined = combined.or(zone.willContain(pose, fieldSpeeds, dt));
        }

        return combined;
    }
}