// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CowCatcherConstants;


public class CowCatcher extends SubsystemBase {
  /** Creates a new CowCatcher. */
  DoubleSolenoid fullExtend_Pistion = new DoubleSolenoid(PneumaticsModuleType.REVPH, CowCatcherConstants.pushcatcherFullForwardID, CowCatcherConstants.pushcatcherFullReverseID);
  DoubleSolenoid halfExtend_Pistion = new DoubleSolenoid(PneumaticsModuleType.REVPH, CowCatcherConstants.pushcatcherHalfForwardID, CowCatcherConstants.pushcatcherHalfReverseID);
  public CowCatcher() {
    //fullExtend_Pistion.set(Value.kReverse);
    //halfExtend_Pistion.set(Value.kReverse);
  }

  /**extends the cowcatcher fully*/
  private void toggle_FullPiston() {
    switch(halfExtend_Pistion.get()){
      case kOff:
      halfExtend_Pistion.set(Value.kReverse);
      fullExtend_Pistion.set(Value.kReverse);
      break;
      case kForward:
      halfExtend_Pistion.set(Value.kOff);
      fullExtend_Pistion.set(Value.kOff);
      break;
      default:
      halfExtend_Pistion.set(Value.kOff);
      fullExtend_Pistion.set(Value.kOff);
    }
  }
  /**extends the cowcatcher halfway */
  public void toggle_HalfPiston() {
    switch(halfExtend_Pistion.get()){
      case kOff:
      halfExtend_Pistion.set(Value.kReverse);
      fullExtend_Pistion.set(Value.kOff);
      break;
      case kForward:
      halfExtend_Pistion.set(Value.kOff);
      fullExtend_Pistion.set(Value.kOff);
      break;
      default:
      halfExtend_Pistion.set(Value.kOff);
      fullExtend_Pistion.set(Value.kOff);
    }
  }
  public Command moveToHalf() {
    return run(()-> {
      halfExtend_Pistion.set(Value.kReverse);
    fullExtend_Pistion.set(Value.kOff);
  });
    
  }

  /**
   * moves the cowcatcher half out
   *
   */
  public Command toggle_Half_Command() {
    return runOnce(this::toggle_HalfPiston);
  }
  /**
   * move the cowcatcher all the way out
   *
   */
  public Command toggle_Full_Command() {
    return runOnce(this::toggle_FullPiston);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
