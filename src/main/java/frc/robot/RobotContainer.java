// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Vision;

public class RobotContainer {

  public CommandXboxController controller = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
  Vision visionSub = new Vision();
  Drive driveSub = new Drive(visionSub::getCurrentPoseEstimate);

  public RobotContainer() {
    driveSub.setDefaultCommand(Commands.run(() -> driveSub.ArcadeDrive(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller.getLeftX()), driveSub));
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(Commands.runOnce(driveSub::ShiftC, driveSub));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
