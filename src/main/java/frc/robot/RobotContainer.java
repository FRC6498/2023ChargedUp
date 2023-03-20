// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterOnChargeStation;
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
  CenterOnChargeStation centerOnChargeStation;

  public RobotContainer() {
    PathPlannerServer.startServer(5811);
    System.out.println("Robot Start");

    //puts a dropdown on smartdashboard that lets the drive team pick which auto they want to run
    chooser = new SendableChooser<>();
    chooser.addOption("encoder Auto", driveBackAuto);
    chooser.addOption("Timed Auto", balanceOnChargeStation);
    chooser.addOption("Trajectory Auto", trajectoryAuto);
    SmartDashboard.putData(chooser);

    driveController = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
    operatorController = new CommandXboxController(OperatorConstants.Operator_Controller_ID);

    visionSub = new Vision();
    armSub = new Arm();
    intakeSub = new Intake();
    driveSub = new Drive(visionSub, armSub, intakeSub);
    cowCatcherSub = new CowCatcher();
    centerOnChargeStation = new CenterOnChargeStation(driveSub, 8, 0.33);
    
    
    //Default Commands ---------------------------------------------------------------------------------------------------------------------------
    //sets the command that each subsystem will run if no other commands are given to it
    //! these commands will stop if the subsystem is given any other command to run
    armSub.setDefaultCommand(
      armSub.InitialArmCommand(
        ()->operatorController.getLeftTriggerAxis(), ()->operatorController.getRightTriggerAxis()
      )
    );
    // sets weather to use keyboard or controller (for simulation) 
    if (Robot.isReal() || !isKeyboard) {
      driveSub.setDefaultCommand(
        driveSub.ArcadeDriveCmd(
        () -> driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis(),
        driveController::getLeftX)
      );
    } else if (isKeyboard) {
      driveSub.setDefaultCommand(
        driveSub.ArcadeDriveCmd(driveController::getLeftY, driveController::getLeftX)
      );
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
    driveController.rightBumper().onTrue(driveSub.ShiftCmd());
    // center robot on charge station ----------------------------------------------------------------------------
    //driveController.rightStick().onTrue(null);
    // arm Commands -----------------------------------------------------------------------------------------
    operatorController.a().onTrue(armSub.retractArm());
    operatorController.b().onTrue(armSub.extendArmMidPID());
    operatorController.y().onTrue(armSub.extendArmHighPID());
    // intake speed Commands ---------------------------------------------------------------------------------
    operatorController.rightBumper().onTrue(intakeSub.stopIntakeCmd());
    operatorController.povUp().onTrue(intakeSub.setIntakeSpeedForward50Cmd().until(()->intakeSub.getCurrentLimit() == true)).onFalse(intakeSub.HoldIntake());
    operatorController.povRight().onTrue(intakeSub.setIntakeSpeedForward75Cmd()).onFalse(intakeSub.HoldIntake());
    operatorController.povDown().onTrue(intakeSub.setIntakeSpeedReverse50Cmd()).onFalse(intakeSub.HoldIntake());
    operatorController.povLeft().onTrue(intakeSub.setIntakeSpeedReverse100Cmd()).onFalse(intakeSub.HoldIntake());
    //home arm ---------------------------------------------------------------------------------------------------------
    driveController.povUp().onTrue(armSub.homeArm());
    //centering Command--------------------------------------------------------------------------------------------------
    operatorController.rightStick().onTrue(centerOnChargeStation);
    operatorController.x().onTrue(armSub.extendToPickup());
    operatorController.rightBumper().onTrue(driveSub.setLEDColorCommand(()->0));
    operatorController.leftBumper().onTrue(driveSub.setLEDColorCommand(()->255));

    
  }

  public Command getAutonomousCommand() {
    //sets which auto command to use based on the input from the dropdown on Smartdashboard

   // return Autos.DevPath(driveSub, "TestPath");
    if(chooser.getSelected() == balanceOnChargeStation){
      return Autos.balanceOnChargeStationAuto(driveSub, armSub, intakeSub, centerOnChargeStation);  
    }else if (chooser.getSelected() == driveBackAuto) {
      Autos.balanceOnChargeStationAuto(driveSub, armSub, intakeSub, centerOnChargeStation);
    } else if (chooser.getSelected() == trajectoryAuto) {
      return Autos.DevPath(driveSub, driveBackAuto);
    }else {
      return Autos.balanceOnChargeStationAuto(driveSub, armSub, intakeSub, centerOnChargeStation);
    }
    return Autos.balanceOnChargeStationAuto(driveSub, armSub, intakeSub, centerOnChargeStation);
  }

  public Command shiftToHigh() {
    return driveSub.ShiftCmd();
  }

}
