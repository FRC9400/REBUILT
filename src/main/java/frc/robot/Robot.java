// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean built = false;

  @Override
  public void robotInit() {
    SignalLogger.setPath("/media/sda1/");

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Can Utilization", RobotController.getCANStatus().percentBusUtilization);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());
    SmartDashboard.updateValues();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (DriverStation.getAlliance().isPresent() && !built) {
      built = true;
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.makeIdle();

    //NEW THING
    // 1. Try to seed from Limelight
    boolean seeded = m_robotContainer.getSwerve().seedGyroFromLimelight();

    // 2. If no tags visible, fall back to the auto's known starting pose
    if (!seeded) {
        m_robotContainer.getAutos().getSelectedStartingPose().ifPresentOrElse(
            pose -> {
                m_robotContainer.getSwerve().resetPose(pose);
                DriverStation.reportWarning("[Auto] Vision failed: reset to auto starting pose", false);
            },
            () -> {
                DriverStation.reportWarning("[Auto] Vision failed: no starting pose, zeroing gyro", false);
                m_robotContainer.getSwerve().zeroGyro();
            }
        );
    }

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      //m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.makeIdle();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
