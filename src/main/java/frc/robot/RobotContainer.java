// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterOnShelf;
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
  Intake intakeSub;
  SendableChooser<String> chooser;
  String driveBackAuto = "drive back";
  String balanceOnChargeStation = "balance on charge station";
  String trajectoryAuto = "follow trajectry (**untested**)";
  private boolean isKeyboard = false;
  CenterOnShelf centerOnChargeStation;

  public RobotContainer() {
    PathPlannerServer.startServer(5811);
    System.out.println("Robot Start");
    chooser = new SendableChooser<>();
    chooser.addOption("encoder Auto", driveBackAuto);
    chooser.addOption("Timed Auto", balanceOnChargeStation);
    chooser.addOption("Trajectory Auto", trajectoryAuto);

    driveController = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
    operatorController = new CommandXboxController(OperatorConstants.Operator_Controller_ID);
    visionSub = new Vision();
    armSub = new Arm();
    intakeSub = new Intake();
    driveSub = new Drive(visionSub, armSub, intakeSub);
    cowCatcherSub = new CowCatcher();
    centerOnChargeStation = new CenterOnShelf(driveSub);
    
    
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
    //driveController.b().onTrue(cowCatcherSub.toggle_Full_Command());
    //driveController.a().onTrue(cowCatcherSub.toggle_Half_Command());
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
    operatorController.rightBumper().onTrue(intakeSub.stopIntake());
    operatorController.povUp().onTrue(intakeSub.setIntakeSpeedForward50()).onFalse(intakeSub.stopIntake());
    operatorController.povRight().onTrue(intakeSub.setIntakeSpeedForward100()).onFalse(intakeSub.stopIntake());
    operatorController.povDown().onTrue(intakeSub.setIntakeSpeedReverse50()).onFalse(intakeSub.stopIntake());
    operatorController.povLeft().onTrue(intakeSub.setIntakeSpeedReverse100()).onFalse(intakeSub.stopIntake());
    //home arm ---------------------------------------------------------------------------------------------------------
    driveController.povUp().onTrue(armSub.homeArm());
    //centering Command--------------------------------------------------------------------------------------------------
    operatorController.rightStick().onTrue(centerOnChargeStation);
    operatorController.x().onTrue(armSub.extendToPickup());
    operatorController.rightBumper().onTrue(driveSub.setLEDColorCommand(()->1));
    operatorController.leftBumper().onTrue(driveSub.setLEDColorCommand(()->255));
    
  }

  public Command getAutonomousCommand() {
   // return Autos.DevPath(driveSub, "TestPath");
     if(chooser.getSelected() == balanceOnChargeStation){
      return Autos.balanceOnChargeStationAuto(driveSub);  
    }else if (chooser.getSelected() == driveBackAuto) {
      Autos.balanceOnChargeStationAuto(driveSub);
    } else if (chooser.getSelected() == trajectoryAuto) {
      return Autos.DevPath(driveSub, driveBackAuto);
    }else {
      //return  Autos.balanceOnChargeStationAuto(driveSub);
      return Autos.balanceOnChargeStationAuto(driveSub);
    }
    return  Autos.balanceOnChargeStationAuto(driveSub);//Autos.TimeBasedAuto1(driveSub, armSub);
  }
  public Command shiftToHigh() {
    return driveSub.Shift();
  }

}
