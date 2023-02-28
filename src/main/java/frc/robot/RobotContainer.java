// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autos.Autos;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CowCatcher;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Vision;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public class RobotContainer implements Loggable {


  public CommandXboxController driveController, operatorController;

  Vision visionSub;
  Drive driveSub;
  Arm armSub;
  CowCatcher cowCatcherSub;

  private boolean isKeyboard = false;

  public RobotContainer() {
    PathPlannerServer.startServer(5811);
    System.out.println("Robot Start");

    driveController = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
    operatorController = new CommandXboxController(OperatorConstants.Operator_Controller_ID);
    visionSub = new Vision();
    driveSub = new Drive(visionSub);
    cowCatcherSub = new CowCatcher();
    armSub = new Arm();
    // arm.setDefaultCommand(arm.homeArmX());

    Logger.configureLoggingAndConfig(this, false);
    configureBindings();
  }

  private void configureBindings() {
    // toggle cowcatcher
    driveController.b().onTrue(cowCatcherSub.toggle_Half_Command());
    driveController.a().onTrue(cowCatcherSub.toggle_Full_Command());
    // toggle breaks
    driveController.leftBumper().onTrue(driveSub.toggleBreak());
    // shift
    driveController.rightBumper().onTrue(driveSub.Shift());
    // center robot
    driveController.rightStick().onTrue(driveSub.centerOnChargeStation());
    // arm Commands
    operatorController.a().onTrue(armSub.DeployArm());
    operatorController.b().onTrue(armSub.RetractArm());
    // intake commands
    operatorController.rightBumper().onTrue(armSub.stopIntake());
    operatorController.povUp().onTrue(armSub.setIntakeSpeedForward25()).onFalse(armSub.stopIntake());
    operatorController.povRight().onTrue(armSub.setIntakeSpeedForward75()).onFalse(armSub.stopIntake());
    operatorController.povDown().onTrue(armSub.setIntakeSpeedReverse25()).onFalse(armSub.stopIntake());
    operatorController.povLeft().onTrue(armSub.setIntakeSpeedReverse75()).onFalse(armSub.stopIntake());

    // sets weather to use keyboard or controller
    if (Robot.isReal() || !isKeyboard) {
      driveSub.setDefaultCommand(driveSub.ArcadeDrive(
          () -> driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis(),
          driveController::getLeftX));
    } else if (isKeyboard) {
      driveSub.setDefaultCommand(
          driveSub.ArcadeDrive(driveController::getLeftY, driveController::getLeftX));
    }
  }

  public Command getAutonomousCommand() {
    return Autos.DevPath(driveSub, "TestPath");
  }

}
