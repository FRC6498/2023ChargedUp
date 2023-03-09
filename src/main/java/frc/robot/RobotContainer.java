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
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Vision;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public class RobotContainer implements Loggable {


  public CommandXboxController driveController, operatorController;

  Vision visionSub;
  Drive driveSub;
  Arm armSub;
  CowCatcher cowCatcherSub;
  Intake intake;

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
    intake = new Intake();
    
    //Default Commands ---------------------------------------------------------------------------------------------------------------------------
    armSub.setDefaultCommand(armSub.InitialArmCommand(()->operatorController.getLeftTriggerAxis(), ()->operatorController.getRightTriggerAxis()));
    // sets weather to use keyboard or controller (for simulation) 
    if (Robot.isReal() || !isKeyboard) {
      driveSub.setDefaultCommand(driveSub.ArcadeDrive(
      () -> driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis(),
     driveController::getLeftX));
    } else if (isKeyboard) {
     driveSub.setDefaultCommand(
      driveSub.ArcadeDrive(driveController::getLeftY, driveController::getLeftX));
    }
    //Logger config -------------------------------------------------------------------------------------------------------------
    Logger.configureLoggingAndConfig(this, false); 
    configureBindings();
  }

  private void configureBindings() {
    // cowcatcher commands ---------------------------------------------------------------------------------
    driveController.b().onTrue(cowCatcherSub.toggle_Full_Command());
    driveController.a().onTrue(cowCatcherSub.toggle_Half_Command());
   // driveController.y().onTrue(driveSub.centerOnChargeStation());
    // drive Controlls ----------------------------------------------------------------------------------
    driveController.leftBumper().onTrue(driveSub.toggleBreak());
    driveController.rightBumper().onTrue(driveSub.Shift());
    // center robot on charge station ----------------------------------------------------------------------------
    //driveController.rightStick().onTrue(null);
    // arm Commands -----------------------------------------------------------------------------------------
    operatorController.a().onTrue(armSub.retractArm());
    operatorController.b().onTrue(armSub.extendArmMidPID());
    operatorController.y().onTrue(armSub.extendArmHighPID());
    // intake speed Commands ---------------------------------------------------------------------------------
    operatorController.rightBumper().onTrue(intake.stopIntake());
    operatorController.povUp().onTrue(intake.setIntakeSpeedForward50()).onFalse(intake.stopIntake());
    operatorController.povRight().onTrue(intake.setIntakeSpeedForward100()).onFalse(intake.stopIntake());
    operatorController.povDown().onTrue(intake.setIntakeSpeedReverse50()).onFalse(intake.stopIntake());
    operatorController.povLeft().onTrue(intake.setIntakeSpeedReverse100()).onFalse(intake.stopIntake());
    //home arm ---------------------------------------------------------------------------------------------------------
    driveController.povUp().onTrue(armSub.homeArm());
    //centering Command--------------------------------------------------------------------------------------------------
    operatorController.rightStick().onTrue(driveSub.centerOnChargeStation());
    
  }

  public Command getAutonomousCommand() {
    return Autos.DevPath(driveSub, "TestPath");
  }

}
