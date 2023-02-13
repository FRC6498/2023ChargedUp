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
  DoubleSolenoid fullExtend_Pistion = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CowCatcherConstants.pushcatcherForwardID, CowCatcherConstants.pushcatcherReverseID);
  DoubleSolenoid halfExtend_Pistion = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CowCatcherConstants.pushcatcherForwardID, CowCatcherConstants.pushcatcherReverseID);
  public CowCatcher() {}

  /**extends the cowcatcher fully*/
  private void toggle_FullPiston() {
    switch(fullExtend_Pistion.get()){
      case kForward:
      fullExtend_Pistion.set(Value.kReverse);
      halfExtend_Pistion.set(Value.kOff);
      break;
      case kReverse:
      fullExtend_Pistion.set(Value.kForward);
      halfExtend_Pistion.set(Value.kOff);
      break;
      default:
      fullExtend_Pistion.set(Value.kReverse);
      halfExtend_Pistion.set(Value.kOff);
      break;
    }
  }
  /**extends the cowcatcher halfway */
  public void toggle_HalfPiston() {
    switch(halfExtend_Pistion.get()){
      case kForward:
      halfExtend_Pistion.set(Value.kReverse);
      fullExtend_Pistion.set(Value.kOff);
      break;
      case kReverse:
      halfExtend_Pistion.set(Value.kForward);
      fullExtend_Pistion.set(Value.kOff);
      break;
      default:
      halfExtend_Pistion.set(Value.kReverse);
      fullExtend_Pistion.set(Value.kOff);
    }
  }
  /**
   * moves the cowcatcher half out
   * 
   */
  public Command toggle_Half() {
    return runOnce(this::toggle_Full);
  }
  /**
   * move the cowcatcher all the way out
   * 
   */
  public Command toggle_Full() {
    return runOnce(this::toggle_Half);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

