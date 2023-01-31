// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CowCatcherConstants;


public class CowCatcher extends SubsystemBase {
  /** Creates a new CowCatcher. */
  DoubleSolenoid pushcatcher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CowCatcherConstants.pushcatcherForwardID, CowCatcherConstants.pushcatcherReverseID);
 
  public CowCatcher() {}

  /**toggles the pushcatcher */
  private void toggle() {
    switch(pushcatcher.get()){
      case kForward:
      pushcatcher.set(Value.kReverse);
      break;
      case kReverse:
      pushcatcher.set(Value.kForward);
      break;
      default:
      pushcatcher.set(Value.kForward);
      break;
    }
  }

  public Command togglePushCatcher() {
    return runOnce(this::toggle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

