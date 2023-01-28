// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utility.NTHelper;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    System.out.println("Robot Ctor!");
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    addPeriodic(m_robotContainer.visionSub::run, kDefaultPeriod);
    NTHelper.sendTagLayout(VisionConstants.tagLayout);
    System.out.println("RobotInit!");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    System.out.println("Robot Periodic!");
  }

  @Override
  public void disabledInit() {
    System.out.println("DisabledInit!");
  }

  @Override
  public void disabledPeriodic() {
    System.out.println("Disabled Periodic!");
  }

  @Override
  public void disabledExit() {
    System.out.println("Disabled Exit!");
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    System.out.println("Auto Init!");
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("Auto Periodic!");
  }

  @Override
  public void autonomousExit() {
    System.out.println("Auto Exit!");
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    System.out.println("Teleop Init!");
  }

  @Override
  public void teleopPeriodic() {
    System.out.println("Teleop Periodic!");
  }

  @Override
  public void teleopExit() {
    System.out.println("Teleop Exit!");
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    System.out.println("Test Init!");
  }

  @Override
  public void testPeriodic() {
    System.out.println("test Periodic!");
  }

  @Override
  public void testExit() {
    System.out.println("Test Exit!");
  }
}
