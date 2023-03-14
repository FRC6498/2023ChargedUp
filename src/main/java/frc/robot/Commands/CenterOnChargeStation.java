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
  double maxSpeed;
  public CenterOnChargeStation(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0;
    maxSpeed = 0.33;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(Math.abs(drive.get1Pitch()) <8)) {
      speed =maxSpeed ;
    } else{
      speed = 0;
    }
    if (drive.get1Pitch() < 0)
    {
      speed = -speed;
    }
    drive.ArcadeDriveCmd(()->speed,()-> 0); 
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
