package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// These comments will give you a peek into my sanity as it goes down the drain.
public class Climb extends SubsystemBase {
    private final TalonFX leftClimbMotor = new TalonFX(ClimbConstants.LEFT_CLIMB_MOTOR_ID, "rio");
    private final TalonFX rightClimbMotor = new TalonFX(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, "rio");
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public Climb() {
        MotorOutputConfigs LeftMotorConfigs = new MotorOutputConfigs();
        LeftMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        LeftMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        MotorOutputConfigs RightMotorConfigs = new MotorOutputConfigs();
        RightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        RightMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.GravityType = GravityTypeValue.Climb_Static;
        slot0.kS = ClimbConstants.kS;
        slot0.kG = ClimbConstants.kG;
        slot0.kV = ClimbConstants.kV;
        slot0.kA = ClimbConstants.kA;
        slot0.kP = ClimbConstants.kP;
        slot0.kI = ClimbConstants.kI;
        slot0.kD = ClimbConstants.kD;
    }

}
