// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SysIdMechanism;
import io.github.oblarg.oblog.Logger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static boolean sysidActive = true;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    addPeriodic(m_robotContainer.visionSub::run, kDefaultPeriod);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
  }

  @Override
  public void disabledInit() {
    if (sysidActive) {
      if (m_robotContainer.sysidMech != null) {
        SysIdMechanism mech = (SysIdMechanism)m_robotContainer.mechChooser.getSelected();
        m_robotContainer.sysidMech.setMotorControllers(0, List.of(mech.getMotor()));
        m_robotContainer.sysidMech.sendData();
      } else if (m_robotContainer.sysidDrive != null) {
        ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>(m_robotContainer.driveSub.getLeftMotors());
        //motors.addAll(m_robotContainer.driveSub.getRightMotors());
        for (WPI_TalonFX motorFx : m_robotContainer.driveSub.getRightMotors()) {
          motors.add(motorFx);
        }
        m_robotContainer.sysidDrive.setMotorControllers(0, motors);
        m_robotContainer.sysidDrive.sendData();
      }
      
    }
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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
