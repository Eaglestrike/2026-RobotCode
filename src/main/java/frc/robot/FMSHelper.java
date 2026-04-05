package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FMSHelper {
    private static Optional<Alliance> alliance = DriverStation.getAlliance();

    public static Trigger activeHub(DoubleSupplier tofSupplier) {
        return new Trigger(() -> isHubActive(tofSupplier.getAsDouble()));
    }

    public static Trigger almostActiveHub(DoubleSupplier tofSupplier) {
        return new Trigger(() -> isAlmostHubActive(tofSupplier.getAsDouble()));
    }

    private static boolean isAlmostHubActive(double tof) {
        alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty() || matchTime < 0l) {
            return false;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return false;
            }
        }

        // Shift 1 is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        boolean futureActive = evaluateHub(matchTime - 2*tof, shift1Active);

        return futureActive;
    }

    /**
     * Determines whether our alliance's hub is currently active
     */
    private static boolean isHubActive(double tof) {
        alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty() || matchTime < 0l) {
            return false;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return false;
            }
        }

        // Shift 1 is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        DogLog.forceNt.log("FMS/Match Time", matchTime);
        DogLog.log("Who Active First", redInactiveFirst ? "Blue" : "Red");

        boolean currentlyActive = evaluateHub(matchTime, shift1Active);
        boolean futureActive = evaluateHub(matchTime - tof, shift1Active);
        
        return currentlyActive || futureActive;
    }

    private static boolean evaluateHub(double matchTime, boolean shift1Active) {
        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

}
