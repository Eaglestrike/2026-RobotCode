package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;

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

    // Ball speed measurement
    private final DigitalInput sensor1Input = new DigitalInput(0); // DIO 0
    private final DigitalInput sensor2Input = new DigitalInput(1); // DIO 1
    private final SynchronousInterrupt sensor1 = new SynchronousInterrupt(sensor1Input);
    private final SynchronousInterrupt sensor2 = new SynchronousInterrupt(sensor2Input);
    private double lastBallSpeed = Double.NaN;
    private double lastTime1 = Double.NaN;
    private double lastTime2 = Double.NaN;

    // Single-sensor value placeholders
    private double lastProcessedRise1 = 0;
    private double lastProcessedRise2 = 0;
    private double lastProcessedFall2 = 0; // For two-beam calc

    private boolean manualTestMode = false;

    public Shooter(EagleSwerveDrivetrain drive) {
        turret = new ShooterTurret();
        hood = new ShooterHood();
        flywheels = new ShooterFlywheels();
        controller = new ShooterController(drive, turret, hood, flywheels);

        // Configure interrupts for beambreak sensors - both edges for duration calc
        sensor1.setInterruptEdges(true, true); // rising (exit) and falling (enter)
        sensor2.setInterruptEdges(true, true); // rising (exit) and falling (enter)

        // Shooter Kill Logic
        Command killShooterCmd = Commands.run(() -> {
            turret.stop().schedule();
            hood.stop().schedule();
            flywheels.stop().schedule();
        }, turret, hood, flywheels).withName("ShooterKill");

        DogLog.tunable("ShooterKill", false, val -> {
            if (val) {
                killShooterCmd.schedule();
            } else {
                killShooterCmd.cancel();
            }
        });
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
                flywheels.stop());
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

        // Ball speed calculation

        // --- Sensor 1 ---
        // getRisingTimestamp = beam exit (end of blockage)
        // getFallingTimestamp = beam enter (start of blockage)
        double s1Rise = sensor1.getRisingTimestamp();
        double s1Fall = sensor1.getFallingTimestamp();

        // Single beam speed calc (Sensor 1)
        // Check if we have a valid pulse (End > Start) and it is NEW (End > last
        // processed End)
        if (!Double.isNaN(s1Rise) && !Double.isNaN(s1Fall) &&
                s1Rise > s1Fall &&
                s1Rise > lastProcessedRise1) {

            double duration = s1Rise - s1Fall;
            // Prevent division by zero or extremely small duration noise
            if (duration > 0.0001) {
                double speed = ShooterBallSpeedConstants.GAME_PIECE_DIAMETER / duration;
                DogLog.log("ShooterBallSpeed/SingleBeam1", speed, "m/s");
            }
            lastProcessedRise1 = s1Rise;
        }

        // --- Sensor 2 ---
        double s2Rise = sensor2.getRisingTimestamp();
        double s2Fall = sensor2.getFallingTimestamp();

        // Single beam speed calc (Sensor 2)
        if (!Double.isNaN(s2Rise) && !Double.isNaN(s2Fall) &&
                s2Rise > s2Fall &&
                s2Rise > lastProcessedRise2) {

            double duration = s2Rise - s2Fall;
            if (duration > 0.0001) {
                double speed = ShooterBallSpeedConstants.GAME_PIECE_DIAMETER / duration;
                DogLog.log("ShooterBallSpeed/SingleBeam2", speed, "m/s");
            }
            lastProcessedRise2 = s2Rise;
        }

        // --- Two Beam Calculation (using falling edges / beam break start) ---
        // Only calculate if both sensors have been triggered this cycle or recently
        // and timestamps indicate a sequence 1 -> 2
        double time1 = s1Fall; // Use same timestamp captured above
        double time2 = s2Fall;

        // Check if sequence is correct (1 -> 2) AND we haven't processed this specific
        // Sensor 2 event yet
        if (!Double.isNaN(time1) && !Double.isNaN(time2)
                && time2 > time1
                && time2 > lastProcessedFall2) {
            double timeDiff = time2 - time1; // seconds
            if (timeDiff > 0.0001) {
                double speed = ShooterBallSpeedConstants.DISTANCE_BETWEEN_SENSORS / timeDiff; // m/s
                lastBallSpeed = speed;
                DogLog.log("ShooterBallSpeed/TwoBeam", speed, "m/s");
            }
            lastProcessedFall2 = time2;
        } // Log beambreak sensor state (true = beam broken)
        boolean beam1Broken = !sensor1Input.get();
        boolean beam2Broken = !sensor2Input.get();
        DogLog.log("Shooter/BeamBreak1", beam1Broken);
        DogLog.log("Shooter/BeamBreak2", beam2Broken);
    }
}
