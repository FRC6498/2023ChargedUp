// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utility.Conversions;

public class Arm extends SubsystemBase {
 ArmFeedforward armFeedforward = new ArmFeedforward(0.39958, 0.012185, 0.00032121);
  CANSparkMax intake;
  WPI_TalonFX slideMotor, armExtensionMotor;
  Trigger slideMotorLeftLimit, slideMotorRightLimit, armExtensionTopLimit, armExtensionBottomLimit;
  boolean intakeRunning;
  double slideMotorMaxDistance, extensionMotorMaxDistance;

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
    intake = new CANSparkMax(ArmConstants.IntakeSpark_ID, MotorType.kBrushless);
    intakeRunning = false;

    slideMotorLeftLimit = new Trigger(this::getSlideLeftLimit);
    slideMotorRightLimit = new Trigger(this::getSlideRightLimit);
    armExtensionTopLimit = new Trigger(this::getExtensionForwardLimit);
    armExtensionBottomLimit = new Trigger(this::getExtensionReverseLimit);
    slideMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


    slideMotor.config_kP(0, 0.2);
    slideMotor.config_kD(0, 0.1);

    armExtensionMotor.setNeutralMode(NeutralMode.Brake);
    intake.setOpenLoopRampRate(0.25);
  }

  public void moveXAxis(double position) {
    slideMotor.set(ControlMode.Position, position);
  }
  public Command manualMoveXAxis(double percent) {
    return run(() -> slideMotor.set(ControlMode.PercentOutput, percent));
  }

  public void moveYAxis(double position) {
    
    slideMotor.setVoltage(armFeedforward.calculate(0, 0));
  }

  public Command Stop() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0));
  }

  public Command moveY(double percent) {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, percent));
  }

  public void moveToTransform(Transform2d transfrom) {
    slideMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getX()));
    armExtensionMotor.set(ControlMode.Position,
        Conversions.distanceToNativeUnits(transfrom.getY()));
  }
  public void armVoltage(double voltage) {
    armExtensionMotor.set(ControlMode.Current, voltage);
  }

  public Command runIntake() {
    return runOnce(() -> {
      if (intakeRunning == true) {
        intake.set(0);
        intakeRunning = false;
      } else {
        intake.set(-0.5);
        intakeRunning = true;
      }
    });
  }

  public Command stopIntake() {
    return run(() -> intake.set(0));
  }

  public Command setIntakeSpeedForward50() {
    return run(() -> intake.set(0.50));
  }

  public Command setIntakeSpeedForward100() {
    return run(() -> intake.set(1));
  }

  public Command setIntakeSpeedReverse50() {
    return run(() -> intake.set(-0.50));
  }

  public Command setIntakeSpeedReverse100() {
    return run(() -> intake.set(-1));
  }

  public Command homeArmX() {
    return run(() -> slideMotor.set(ControlMode.PercentOutput, 0.5))
        .until(() -> slideMotorLeftLimit.getAsBoolean() == true)
        .andThen(runOnce(
            () -> SmartDashboard.putNumber("X axis pos", slideMotor.getSelectedSensorPosition())),
            runOnce(() -> slideMotor.setSelectedSensorPosition(0)),
            run(() -> slideMotor.set(ControlMode.PercentOutput, -0.5))
                .until(() -> slideMotorRightLimit.getAsBoolean() == true),
            // x has hit right limit
            runOnce(() -> slideMotorMaxDistance = slideMotor.getSelectedSensorPosition()),
            runOnce(() -> SmartDashboard.putNumber("X Axis Max", slideMotorMaxDistance)),
            run(() -> moveXAxis(slideMotorMaxDistance / 2)));
  }

  public Command centerOnTarget(Transform2d robotToTarget) {
    return run(() -> moveToTransform(robotToTarget));
  }

  public Command DeployArm() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, -0.3))
        .until(() -> armExtensionMotor.isRevLimitSwitchClosed()==1)
        .andThen(run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0)).withTimeout(0.25),
        run(() -> armExtensionMotor.set(ControlMode.PercentOutput, -0.07) )
        );
  }

  public Command RetractArm() {
    return run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0.2))
        .until(() -> armExtensionMotor.isFwdLimitSwitchClosed() == 1)
        .andThen(run(() -> armExtensionMotor.set(ControlMode.PercentOutput, 0)));

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
