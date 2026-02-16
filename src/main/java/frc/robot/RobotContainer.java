// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.EagleSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ballistics.BallisticConstants;

/**
 * RobotContainer — 射手子系统测试模式
 * 仅保留 Turret / Flywheel / Hood，其他子系统全部禁用。
 */
public class RobotContainer {
    public static final CANBus MECHANISM_CANBUS = new CANBus("Mechanisms");

    // ─── Drivetrain: 仅创建对象（Shooter 构造需要），不绑定任何命令 ───
    private final EagleSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // ─── Controller ───
    private final CommandXboxController operatorController = new CommandXboxController(2);

    // ─── 只保留射手子系统 ───
    private final Shooter shooter = new Shooter(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // ═══════════════════════════════════════════════
        //  Flywheel 测试
        // ═══════════════════════════════════════════════
        // A 按钮 (按住): 飞轮中档 RPM
        operatorController.a()
                .onTrue(Commands.runOnce(() -> shooter.setManualTestMode(true))
                        .andThen(shooter.getFlywheels().startAtVelocity(BallisticConstants.MEDIUM_SPEED_RPM)))
                .onFalse(Commands.runOnce(() -> shooter.setManualTestMode(false))
                        .andThen(shooter.getFlywheels().stop()));

        // B 按钮 (按住): 飞轮低档 RPM
        operatorController.b()
                .onTrue(Commands.runOnce(() -> shooter.setManualTestMode(true))
                        .andThen(shooter.getFlywheels().startAtVelocity(BallisticConstants.LOW_SPEED_RPM)))
                .onFalse(Commands.runOnce(() -> shooter.setManualTestMode(false))
                        .andThen(shooter.getFlywheels().stop()));

        // ═══════════════════════════════════════════════
        //  Turret 测试
        // ═══════════════════════════════════════════════
        // X 按钮: turret → 45°
        operatorController.x()
                .onTrue(Commands.runOnce(() -> shooter.setManualTestMode(true))
                        .andThen(shooter.getTurret().requestPosition(Degrees.of(45))));

        // Y 按钮: turret → 0°
        operatorController.y()
                .onTrue(Commands.runOnce(() -> shooter.setManualTestMode(true))
                        .andThen(shooter.getTurret().requestPosition(Degrees.of(0))));

        // ═══════════════════════════════════════════════
        //  Hood 测试 — POV 上/下 ±10°
        // ═══════════════════════════════════════════════
        operatorController.povUp()
                .onTrue(Commands.runOnce(() -> shooter.setManualTestMode(true))
                        .andThen(shooter.getHood().adjustGoal(10)));

        operatorController.povDown()
                .onTrue(Commands.runOnce(() -> shooter.setManualTestMode(true))
                        .andThen(shooter.getHood().adjustGoal(-10)));

        // ═══════════════════════════════════════════════
        //  Turret / Hood Zero (Back + X / Back + Y)
        // ═══════════════════════════════════════════════
        operatorController.button(7).and(operatorController.x())
                .onTrue(shooter.getHood().zero().ignoringDisable(true));

        operatorController.button(7).and(operatorController.y())
                .onTrue(shooter.getTurret().zero().ignoringDisable(true));

        // ═══════════════════════════════════════════════
        //  Start 按钮: stow (turret 归零 + hood 最低 + flywheel 停)
        // ═══════════════════════════════════════════════
        operatorController.start()
                .onTrue(shooter.stowShooter());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public Command getTeleopCommand() {
        return Commands.none();
    }

    public EagleSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
}
