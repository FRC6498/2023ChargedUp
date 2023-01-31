// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utility.Conversions;


public class Arm extends SubsystemBase {

  WPI_TalonFX armTalonFX = new WPI_TalonFX(ArmConstants.ArmTalonID);
  ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
 

  public Arm() {
    armTalonFX.configFactoryDefault();
    armTalonFX.config_kP(0, 0.5);
    armTalonFX.config_kD(0, 0.2);
    armTalonFX.setNeutralMode(NeutralMode.Brake);
  }
  /**
   * moves the arm to the given setpoint in degrees
   * @param setpoint
   * in degrees
   */
  public Command moveToDegrees(double setpoint) {
   return run(()->armTalonFX.set(ControlMode.Position, Conversions.DegreesToNativeUnits(setpoint, ArmConstants.ArmGearRatio)));
  }



@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
