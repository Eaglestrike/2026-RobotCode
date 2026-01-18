package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    public enum ShootMode {
        FERRY,
        SHOOT,
    }

    private ShootMode currentMode;
    private boolean isShooting;

    /**
     * TODO: Modify the constructor to initialize using all the other subsystems
     */
    public Superstructure() {
        currentMode = ShootMode.SHOOT;
        isShooting = false;
    }

    /**
     * @return runOnce command that schedules the intake to drop
     */
    public Command intake() {
        return Commands.none();
    }

    public Command stow() {
        return Commands.none();
    }

    /**
     * Command to climb to a specified level
     * 
     * @param level The level to climb to (REPLACE OBJECT WITH ACTUAL TYPE -
     *              PROBABLY INT OR ENUM)
     * @return SequenceCommand that performs the climbing action
     */
    public Command climb(Object level) {
        return Commands.none();
    }

    public Object getClimbState() {
        return null;
    }

    public void setMode(ShootMode mode) {
        currentMode = mode;
    }

    /**
     * Turns shooting on or off
     */
    public void setShooting(boolean shooting) {
        isShooting = shooting;
    }

    @Override
    public void periodic() {
        // If shooting, then periodically recalibrate shooter (turret, hood and flywheel speeds)
    }
}