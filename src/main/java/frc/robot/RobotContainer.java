// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ShootMode;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveTelemetry;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * DriveConstants.DRIVE_DEADBAND_MULT)
			.withRotationalDeadband(MaxAngularRate * DriveConstants.DRIVE_DEADBAND_MULT)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);
	public final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final Vision vision = new Vision(drivetrain);

	// Subsystems
	private final Superstructure superstructure = new Superstructure();

	public RobotContainer() {
		configureBindings();

		NamedCommands.registerCommand("ClimbL1", superstructure.climb(1));
		NamedCommands.registerCommand("SetModeFerry", Commands.runOnce(() -> superstructure.setMode(ShootMode.FERRY)));
		NamedCommands.registerCommand("SetModeShoot", Commands.runOnce(() -> superstructure.setMode(ShootMode.SHOOT)));
		NamedCommands.registerCommand("StartShooting", Commands.runOnce(() -> superstructure.setShooting(true)));
		NamedCommands.registerCommand("StopShooting", Commands.runOnce(() -> superstructure.setShooting(false)));
		NamedCommands.registerCommand("Intake", superstructure.intake());
		NamedCommands.registerCommand("StowIntake", superstructure.stowIntake());
		NamedCommands.registerCommand("Stow", superstructure.stow());
		NamedCommands.registerCommand("AlignLeftTower", drivetrain.alignPID(null));
		NamedCommands.registerCommand("AlignRightTower", drivetrain.alignPID(null));

	}

	private void configureBindings() {
	}

	public Command getOffTower() {
		if(superstructure.getClimbState() == null) {
			return Commands.print("Not currently climbing, cannot get off tower");
		}
		
		int direction = drivetrain.getState().Pose.getY() > 4 ? 1 : -1; // Replace 4 with 1/2 field width
		return Commands.sequence(
				superstructure.climb(0),
				Commands.runOnce(() -> drivetrain.setControl(
						drive.withVelocityY(MaxSpeed * direction * DriveConstants.GET_OFF_TOWER_SPEED_MULT))),
				Commands.waitSeconds(0.5),
				Commands.runOnce(() -> drivetrain.setControl(drive.withVelocityY(0.0))));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
