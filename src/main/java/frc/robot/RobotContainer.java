// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autos.MoveForward1MeterAndLeft1MeterAuto;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Vision;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public class RobotContainer implements Loggable {
  private boolean isKeyboard = true;
  public CommandXboxController controller;
  Vision visionSub;
  Drive driveSub;

  public RobotContainer() {
    controller = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
    visionSub = new Vision();
    driveSub = new Drive(visionSub);
    Logger.configureLoggingAndConfig(this, false);
    configureBindings();
  }

  private void configureBindings() {
    //shifts gears
    controller.a().onTrue(driveSub.Shift());
    // drives
    // if (Robot.isReal() || !isKeyboard) {
      driveSub.setDefaultCommand(driveSub.ArcadeDrive(() -> controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller::getLeftX));
    // } else if(isKeyboard) {
    //   driveSub.setDefaultCommand(driveSub.ArcadeDrive(controller::getLeftY, controller::getLeftX));
    // }
  }

  public Command getAutonomousCommand() {
    return driveSub.ArcadeDrive(() -> 0, () -> 0.5);//new MoveForward1MeterAndLeft1MeterAuto(driveSub);//MoveForward3MetersAuto(driveSub);
  }
}
