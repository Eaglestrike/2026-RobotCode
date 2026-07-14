// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Milliseconds;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

import frc.robot.lib.zone.RebuiltZones;
import org.ironmaple.simulation.SimulatedArena;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withLogExtras(true));
        m_robotContainer = new RobotContainer();
        addPeriodic(m_robotContainer.getSuperstructure()::fastPeriodic, .005);
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
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {
    }

    @Override
    public void utilityExit() {
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
