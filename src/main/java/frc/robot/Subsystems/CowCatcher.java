// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CowCatcherConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class CowCatcher extends SubsystemBase implements Loggable {
  DoubleSolenoid fullExtendPistion;
  DoubleSolenoid halfExtendPistion;
  @Log
  public boolean isOut; 


  public CowCatcher() {
    fullExtendPistion = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, // we use the pcm from ctre on the robot
        CowCatcherConstants.pushcatcherFullForwardID, CowCatcherConstants.pushcatcherFullReverseID); // ports that the solenoids are connected to on the pcm 
    halfExtendPistion = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
        CowCatcherConstants.pushcatcherHalfForwardID, CowCatcherConstants.pushcatcherHalfReverseID);
      isOut = false;
      }

  public Command moveToHalf() {
    return run(() -> {
        halfExtendPistion.set(Value.kReverse);
        fullExtendPistion.set(Value.kOff);
        isOut = true;
      }
    );
  }
  /**
   * toggles the cowcatcher between half-out and fully in
   */
  public Command toggle_Half_Command() {
    return runOnce(() -> {
        switch (halfExtendPistion.get()) {
          case kOff:
            halfExtendPistion.set(Value.kReverse);
            fullExtendPistion.set(Value.kOff);
            isOut = true;
            break;
          case kForward:
            halfExtendPistion.set(Value.kOff);
            fullExtendPistion.set(Value.kOff);
            isOut = false;
            break;
          default:
            halfExtendPistion.set(Value.kOff);
            fullExtendPistion.set(Value.kOff);
            isOut = false;
        }
      }
    );
  }
  /**
   * toggles the cowcatcher between all the way out and all the way in
   */
  public Command toggle_Full_Command() {
    return runOnce(() -> {
        switch (halfExtendPistion.get()) {
          case kOff:
            halfExtendPistion.set(Value.kReverse);
            fullExtendPistion.set(Value.kReverse);
            isOut = true;
            break;
          case kReverse:
            halfExtendPistion.set(Value.kOff);
            fullExtendPistion.set(Value.kOff);
            isOut = false;
            break;
          default:
            halfExtendPistion.set(Value.kOff);
            fullExtendPistion.set(Value.kOff);
            isOut = false;
        }
      }
    );
  }

  @Override
  public void periodic() {
  }
}
