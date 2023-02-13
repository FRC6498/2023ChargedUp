// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utility.Conversions;


public class Arm extends SubsystemBase {
  // ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
  CANSparkMax intake = new CANSparkMax(ArmConstants.IntakeSpark_ID, MotorType.kBrushless);
  TalonFX xAxisMotor = new TalonFX(ArmConstants.xAxisMotorID);
  TalonFX yAxisMotor = new TalonFX(ArmConstants.yAxisMotorID);

  /** functions as limit switch for intake */
  BooleanSupplier currentLimit = new BooleanSupplier() {
    public boolean getAsBoolean() {
      if (ArmConstants.pdh.getCurrent(ArmConstants.ArmPDHPortID) > 10) {
        return true;
      } else {
        return false;
      }
    };
  };

  public Arm() {

  }

  /** Runs the intake on the arm */
  public Command runIntake() {
    return run(() -> intake.set(0.5)).until(currentLimit);
  }

  /** Centering Command */
  public Command centerOnTarget(Transform2d robotToTarget) {
    return run(() -> moveToTransform(robotToTarget));
  }

  /** Centers the robot on a Target given the transform from the robot to the target */
  public void moveToTransform(Transform2d transfrom) {
    xAxisMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getX()));
    yAxisMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getY()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
