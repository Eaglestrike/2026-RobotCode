// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

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

  private static final int MAX_LOG_FILES = 10;
  private static final long MAX_LOG_DIR_BYTES = 500L * 1024 * 1024; // 500 MB

  public Robot() {
    DogLog.setOptions(new DogLogOptions()
        .withCaptureNt(false)
        .withLogExtras(false));
    cleanupOldLogs();
    m_robotContainer = new RobotContainer();
  }

  private void cleanupOldLogs() {
    File logDir = new File(RobotBase.isReal() ? "/home/lvuser/logs" : "logs");
    if (!logDir.exists() || !logDir.isDirectory()) return;

    File[] logs = logDir.listFiles((dir, name) -> name.endsWith(".wpilog"));
    if (logs == null || logs.length <= 1) return;

    Arrays.sort(logs, Comparator.comparingLong(File::lastModified));

    long totalSize = 0;
    for (File f : logs) totalSize += f.length();

    int i = 0;
    while (i < logs.length - 1 && (logs.length - i > MAX_LOG_FILES || totalSize > MAX_LOG_DIR_BYTES)) {
      totalSize -= logs[i].length();
      System.out.println("[LogCleanup] Deleting " + logs[i].getName()
          + " (" + (logs[i].length() / 1024) + " KB)");
      logs[i].delete();
      i++;
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
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
    DogLog.log("Sim/FuelPoses", fuelPoses);
  }
}
