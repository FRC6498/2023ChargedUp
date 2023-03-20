// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive;

public class CenterOnChargeStation extends CommandBase {
  /** Creates a new CenterOnShelf. */
  Drive drive;
  double speed;
  double maxPercentSpeed;
  double tolerance;
   /**
    * command that centers the robot on the charge station
    * @param drive - inject the drive subsystem
    * @param tolerance - tolerance for the command
    * @param maxPercentSpeed - max speed the motors will be set at during the command
    */
  public CenterOnChargeStation(Drive drive, double tolerance, double maxPercentSpeed) {
    addRequirements(drive);
    this.drive = drive;
    this.tolerance = tolerance;
    this.maxPercentSpeed = maxPercentSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(Math.abs(drive.getPitch()) < tolerance)) {
      speed =maxPercentSpeed; // go forward if pitch is not within tolerance and the robot is tipping backward
    } else{
      speed = 0; // stop if robot pitch is within tolerance
    }
    if (drive.getPitch() < 0)
    {
      speed = -speed; //reverse if the robot is tipping forward
    }
    drive.ArcadeDriveCmd(()->speed,()-> 0); // drives the robot at the set speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
