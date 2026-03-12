package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FMSHelper extends SubsystemBase {
    private static Optional<Alliance> alliance = DriverStation.getAlliance();

    public static Trigger activeHub() {
        return new Trigger(FMSHelper::isHubActive);
    }

    /**
     * Determines whether our alliance's hub is currently active
     */
    private static boolean isHubActive() {
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
        if (gameData == null || gameData.isEmpty()) {
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
