// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.CenterOnChargeStation;
import frc.robot.Commands.Autos.Autos;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.CowCatcher;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Intake;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public class RobotContainer implements Loggable {


  public CommandXboxController driveController, operatorController;

  Drive driveSub;
  Arm armSub;
  CowCatcher cowCatcherSub;
  Intake intakeSub;
  SendableChooser<String> chooser;
  String turnAroundAuto = "turnAroundAuto";
  String centerOnChargeStationAuto = "center on charge station";
  String twoPieceAuto = "two piece auto";
  private boolean isKeyboard = false;
  CenterOnChargeStation centerOnChargeStation;

  public RobotContainer() {
    PathPlannerServer.startServer(5811);
    System.out.println("Robot Start");

    //puts a dropdown on smartdashboard that lets the drive team pick which auto they want to run
    chooser = new SendableChooser<>();
    chooser.addOption("turn around auto", turnAroundAuto);
    chooser.addOption("ChargeStation Auto", centerOnChargeStationAuto);
    chooser.addOption("Trajectory Auto", twoPieceAuto);
    SmartDashboard.putData(chooser);

    driveController = new CommandXboxController(OperatorConstants.Driver_Controller_ID);
    operatorController = new CommandXboxController(OperatorConstants.Operator_Controller_ID);
    armSub = new Arm();
    intakeSub = new Intake();
    driveSub = new Drive(armSub, intakeSub);
    cowCatcherSub = new CowCatcher();
    centerOnChargeStation = new CenterOnChargeStation(driveSub);
    VisionConstants.driverCamera  = CameraServer.startAutomaticCapture();
    VisionConstants.driverCamera.setResolution(640, 480);
    VisionConstants.driverCamera.setFPS(30);
    
    
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
    driveController.a().onTrue(cowCatcherSub.toggle_Half_Command());
    driveController.b().onTrue(cowCatcherSub.toggle_Full_Command());
   // driveController.y().onTrue(driveSub.centerOnChargeStation());
    // drive Controlls ----------------------------------------------------------------------------------
    driveController.leftBumper().onTrue(driveSub.toggleBreak());
    driveController.rightBumper().onTrue(driveSub.ShiftCmd());
    // center robot on charge station ----------------------------------------------------------------------------
    //driveController.rightStick().onTrue(null);
    // arm Commands -----------------------------------------------------------------------------------------
    operatorController.a().onTrue(armSub.retactArmPID());
    operatorController.b().onTrue(armSub.extendArmMidPID());
    operatorController.y().onTrue(armSub.extendArmHighPID());
    // intake speed Commands ---------------------------------------------------------------------------------
    operatorController.rightBumper().onTrue(intakeSub.stopIntakeCmd());
    operatorController.povUp().onTrue(intakeSub.setIntakeSpeedForward50Cmd()).onFalse(intakeSub.stopIntakeCmd());
    operatorController.povRight().onTrue(intakeSub.setIntakeSpeedForward75Cmd()).onFalse(intakeSub.stopIntakeCmd());
    operatorController.povDown().onTrue(intakeSub.setIntakeSpeedReverse50Cmd()).onFalse(intakeSub.stopIntakeCmd());
    operatorController.povLeft().onTrue(intakeSub.setIntakeSpeedReverse100Cmd()).onFalse(intakeSub.stopIntakeCmd());
    //home arm ---------------------------------------------------------------------------------------------------------
    driveController.povUp().onTrue(armSub.homeArm());
    //centering Command--------------------------------------------------------------------------------------------------
    operatorController.rightStick().onTrue(centerOnChargeStation);
    operatorController.x().onTrue(armSub.homeSlide());
    operatorController.rightBumper().onTrue(driveSub.setLEDColorCommand(()->0));
    operatorController.leftBumper().onTrue(driveSub.setLEDColorCommand(()->255));

    
  }
    
  public Command getAutonomousCommand() {
    if (chooser.getSelected() == turnAroundAuto) {
      return Autos.driveBackAuto(driveSub, armSub, intakeSub);
    } else if(chooser.getSelected() == centerOnChargeStationAuto) {
      return Autos.balanceOnChargeStationAuto(driveSub, armSub, intakeSub, centerOnChargeStation);
    } else if(chooser.getSelected() == twoPieceAuto) {
      return Autos.Auto1(driveSub, cowCatcherSub, armSub, intakeSub);
    } else {
      return Autos.balanceOnChargeStationAuto(driveSub, armSub, intakeSub, centerOnChargeStation);
    }
  }
}