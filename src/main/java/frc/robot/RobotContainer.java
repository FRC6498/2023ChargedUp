// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CowCatcher;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Vision;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {

  public CommandXboxController controller = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
  Vision visionSub = new Vision();
  Drive driveSub = new Drive(visionSub);
  Arm arm = new Arm();
  CowCatcher cowCatcher = new CowCatcher();

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    driveSub.setDefaultCommand(driveSub.ArcadeDrive(-controller.getLeftX(), controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
    configureBindings();
  }

  private void configureBindings() {
    //shifts gears
    controller.a().onTrue(driveSub.Shift());
    controller.b().onTrue(arm.moveToDegrees(50));
    controller.x().onTrue(cowCatcher.togglePushCatcher());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
