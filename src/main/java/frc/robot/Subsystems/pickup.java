// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * Class based on the sucker prototype from Saturday
 */
public class pickup extends SubsystemBase {
  /** Creates a new pickup. */
  DoubleSolenoid pickerUpper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
  public int pickupPosition;
  public pickup() {
    pickupPosition =1;
    pickerUpper.set(Value.kForward);
  }
  
  /**
   * switches the suction cups on or off
   */
  public void pickupswitch() {
    switch (pickupPosition){

      case 1:
      pickerUpper.set(Value.kReverse);
      pickupPosition = 2;
      break;

      case 2:
      pickerUpper.set(Value.kForward);
      pickupPosition =1;
      break;

      default:
      pickerUpper.set(Value.kForward);
      pickupPosition = 1;
      break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
