package frc.robot.subsystems.climb;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// These comments will give you a peek into my sanity as it goes down the drain.
public class Climb extends SubsystemBase {
    private final TalonFX leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private final TalonFX rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
}
