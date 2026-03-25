// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

import frc.robot.lib.zone.RebuiltZones;
import org.ironmaple.simulation.SimulatedArena;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withLogExtras(true));
        m_robotContainer = new RobotContainer();
        addPeriodic(m_robotContainer.getSuperstructure()::fastPeriodic, Milliseconds.of(5));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
        if (m_robotContainer != null) {
            m_robotContainer.disabledPeriodic();
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().schedule(
                m_robotContainer.getTeleopCommand());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    @Override
    public void simulationPeriodic() {
        Pose3d[] fuelPoses = SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel");
        // DogLog.log("Sim/FuelPoses", fuelPoses);
        RebuiltZones.logAllZones();
    }
}
