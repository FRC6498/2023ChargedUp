// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.SysIdMechanism;
import frc.robot.Subsystems.Vision;
import frc.robot.SysId.Logging.SysIdDrivetrainLogger;
import frc.robot.SysId.Logging.SysIdGeneralMechanismLogger;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {

  public CommandXboxController controller = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
  Vision visionSub = new Vision();
  Drive driveSub = new Drive(visionSub);
  SysIdGeneralMechanismLogger sysidMech;
  SysIdDrivetrainLogger sysidDrive;
  SendableChooser<SubsystemBase> mechChooser;

  public RobotContainer() {
    driveSub.setName("Drive");
    Logger.configureLoggingAndConfig(this, false);
    //driveSub.setDefaultCommand(driveSub.ArcadeDrive(controller.getLeftX(), controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
    driveSub.setDefaultCommand(driveSub.ArcadeDrive(controller::getLeftX, controller::getLeftY));
    mechChooser = new SendableChooser<>();
    //mechChooser.addOption("Drivetrain", driveSub);
    mechChooser.setDefaultOption("Drivetrain", driveSub);
    SmartDashboard.putData(mechChooser);
    configureBindings();
  }

  private void configureBindings() {
    //shifts gears
    controller.a().onTrue(driveSub.Shift());

  }

  public Command getAutonomousCommand() {
    if (Robot.sysidActive) {
      if (mechChooser.getSelected().getName() != "Drive") {
        // not drivetrain
        sysidMech = new SysIdGeneralMechanismLogger();
        SysIdMechanism mech = (SysIdMechanism) mechChooser.getSelected();
        return Commands.runOnce(sysidMech::initLogging, mechChooser.getSelected()).andThen(
          Commands.run(() -> {
            // set desitred voltages and update the data list
            sysidMech.log(sysidMech.measureVoltage(List.of(mech.getMotor())), mech.getPosition(), mech.getVelocity());
            // use desired voltages to update motor setpoints
            sysidMech.setMotorControllers(sysidMech.getMotorVoltage(), List.of(mech.getMotor()));
          }, mechChooser.getSelected())
        );
      } else {
        // drivetrain
        sysidDrive = new SysIdDrivetrainLogger();
        return Commands.runOnce(sysidDrive::initLogging, driveSub).andThen(
          Commands.run(() -> {
            sysidDrive.log(
              sysidDrive.measureVoltage(driveSub.getLeftMotors()),
              sysidDrive.measureVoltage(driveSub.getRightMotors()), 
              driveSub.getLeftDistanceMeters(), 
              driveSub.getRightDistanceMeters(), 
              driveSub.getWheelSpeeds().leftMetersPerSecond, 
              driveSub.getWheelSpeeds().rightMetersPerSecond, 
              driveSub.getGyroAngle().getDegrees(), 
              driveSub.getGyroRate()
            );
            sysidDrive.setMotorControllers(sysidDrive.getLeftMotorVoltage(), driveSub.getLeftMotors());
            sysidDrive.setMotorControllers(sysidDrive.getRightMotorVoltage(), driveSub.getRightMotors());
          }, driveSub)
        );
      }
    } else {
      return Commands.print("No auto selected!");
    }
  }
}
