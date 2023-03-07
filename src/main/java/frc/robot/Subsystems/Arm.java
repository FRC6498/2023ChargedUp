// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utility.Conversions;

public class Arm extends SubsystemBase {
  ArmFeedforward armFeedforward = new ArmFeedforward(0.39958, 0.012185, 0.00032121);

  WPI_TalonFX slideMotor, armExtensionMotor;
  Trigger slideMotorLeftLimit, slideMotorRightLimit, armExtensionTopLimit, armExtensionBottomLimit;

  public double slideMotorMaxDistance, extensionMotorMaxDistance;
  public boolean homeComplete = false;

  /** Current-based limit switch for intake motors */
  BooleanSupplier extensionCurrentLimit = () -> {
    if (ArmConstants.pdh.getCurrent(ArmConstants.ArmPDHPortID) > 25) {
      return true;
    } else {
      return false;
    }
  };

  public Arm() {
    armExtensionMotor = new WPI_TalonFX(ArmConstants.yAxisMotorID);
    slideMotor = new WPI_TalonFX(ArmConstants.xAxisMotorID);
    slideMotorLeftLimit = new Trigger(this::getSlideLeftLimit);
    slideMotorRightLimit = new Trigger(this::getSlideRightLimit);
    armExtensionTopLimit = new Trigger(this::getExtensionForwardLimit);
    armExtensionBottomLimit = new Trigger(this::getExtensionReverseLimit);
    slideMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    slideMotor.config_kP(0, 0.3);
    slideMotor.config_kD(0, 0.2);

    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
    slideMotor.setNeutralMode(NeutralMode.Brake);

  }

  public Command moveArm(DoubleSupplier position) {
    return run(() -> armExtensionMotor.set(ControlMode.Position, position.getAsDouble()));
  }

  public Command manualMoveSlide(DoubleSupplier percent) {
    return run(() -> {
      slideMotor.set(ControlMode.PercentOutput, percent.getAsDouble());
    });
  }

  public Command InitialArmCommand(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    return homeSlide().until(() -> homeComplete == true)
        .andThen(runOnce(() -> this.setDefaultCommand(
            manualMoveSlide(() -> rightTrigger.getAsDouble() - leftTrigger.getAsDouble()))));
  }



  public Command stopArm() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0));
  }

  public Command moveArmPercent(double percent) {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, percent));
  }

  public void moveToTransform(Transform2d transfrom) {
    slideMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getX()));
    armExtensionMotor.set(ControlMode.Position,
        Conversions.distanceToNativeUnits(transfrom.getY()));
  }

  public void setArmVoltage(double voltage) {
    armExtensionMotor.set(ControlMode.Current, voltage);
  }



  public Command homeSlide() {
    return run(() -> slideMotor.set(ControlMode.PercentOutput, 0.5))
        .until(() -> slideMotorLeftLimit.getAsBoolean() == true)
        .andThen(
            runOnce(() -> SmartDashboard.putNumber("X axis pos", slideMotor.getSelectedSensorPosition())),
            runOnce(() -> slideMotor.setSelectedSensorPosition(0)),
            run(() -> slideMotor.set(ControlMode.PercentOutput, -0.5))
                .until(() -> slideMotorRightLimit.getAsBoolean() == true),
            // x has hit right limit
            runOnce(() -> slideMotorMaxDistance = slideMotor.getSelectedSensorPosition()),
            runOnce(() -> SmartDashboard.putNumber("X Axis Max", slideMotorMaxDistance)),
            runOnce(() -> homeComplete = true));

  }

  public Command homeArm() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0.2))
        .until(() -> armExtensionMotor.isFwdLimitSwitchClosed() == 1)
        .andThen(runOnce(() -> armExtensionMotor.setSelectedSensorPosition(0)),
            run(() -> armExtensionMotor.set(ControlMode.PercentOutput, -0.2))
                .until(() -> armExtensionMotor.isRevLimitSwitchClosed() == 1),
            runOnce(
                () -> extensionMotorMaxDistance = armExtensionMotor.getSelectedSensorPosition()),
            run(() -> armExtensionMotor.set(ControlMode.Position, extensionMotorMaxDistance / 1.6))

        );
  }

  public Command centerOnTarget(Transform2d robotToTarget) {
    return run(() -> moveToTransform(robotToTarget));
  }

  public Command extendArm() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, -0.5))
        .until(() -> armExtensionMotor.isRevLimitSwitchClosed() == 1);
  }

  public Command retractArm() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0.5))
        .until(() -> armExtensionMotor.isFwdLimitSwitchClosed() == 1)
        .andThen(() -> slideMotor.set(ControlMode.Position, slideMotorMaxDistance / 2));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // #region getters for limit switches
  public boolean getSlideLeftLimit() {
    if (slideMotor.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getSlideRightLimit() {
    if (slideMotor.isRevLimitSwitchClosed() == 1) {
      return true;
    } else if (slideMotor.isRevLimitSwitchClosed() == 0) {
      return false;
    }
    return false;
  }

  public boolean getExtensionForwardLimit() {
    if (armExtensionMotor.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getExtensionReverseLimit() {
    if (armExtensionMotor.isRevLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }
  // #endregion

}
