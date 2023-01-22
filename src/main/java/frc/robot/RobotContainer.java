// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Vision;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public class RobotContainer implements Loggable {
  private boolean isKeyboard = true;
  public CommandXboxController controller = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
  Vision visionSub = new Vision();
  Drive driveSub = new Drive(visionSub);

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    configureBindings();
  }

  private void configureBindings() {
    //shifts gears
    controller.a().onTrue(driveSub.Shift());
    // drives
    if (Robot.isReal() || !isKeyboard) {
      driveSub.setDefaultCommand(driveSub.ArcadeDrive(() -> controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller::getLeftX));
    } else if(isKeyboard) {
      driveSub.setDefaultCommand(driveSub.ArcadeDrive(controller::getLeftY, controller::getLeftX));
    }
    controller.x().onTrue(driveSub.followTrajectory(TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(), new Pose2d(1, 0, new Rotation2d())), DriveConstants.trajectoryConfig)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
