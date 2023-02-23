// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utility.Conversions;

public class Arm extends SubsystemBase {
  // ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
  CANSparkMax intake;
  TalonFX xAxisMotor, yAxisMotor;
  /** Trigger for limit switch on the arm */
  Trigger xAxisforwardLimitSwitch, xAxisreverseLimitSwitch, yAxisTopLimitSwitch, yAxisBottomLimitSwitch;

  boolean intakeRunning;
  /** Max distance the arm motors can travel */
  double xAxisMotorMax, yAxisMotorMax;

  boolean xHomingComplete, yHomingComplete;

  /** Current-based limit switch for intake motors */
  BooleanSupplier currentLimit = () -> {
    if (ArmConstants.pdh.getCurrent(ArmConstants.ArmPDHPortID) > 10) {
      return true;
    } else {
      return false;
    }
  };

  public Arm() {

    yAxisMotor = new TalonFX(ArmConstants.yAxisMotorID);
    xAxisMotor = new TalonFX(ArmConstants.xAxisMotorID);
    intake = new CANSparkMax(ArmConstants.IntakeSpark_ID, MotorType.kBrushless);

    xAxisforwardLimitSwitch = new Trigger(this::getForwardLimitX);
    xAxisreverseLimitSwitch = new Trigger(this::getReverseLimitX);
    yAxisTopLimitSwitch = new Trigger(this::getForwardLimitY);
    yAxisBottomLimitSwitch = new Trigger(this::getReverseLimitY);
    xAxisMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    intakeRunning = false;
    
    xAxisMotor.config_kP(0, 0.2);
    xAxisMotor.config_kD(0, 0.1);
    
  }

  public void moveXAxis(double position) {
    xAxisMotor.set(ControlMode.Position, position);
  }

  public void moveYAxis(double position) {
    xAxisMotor.set(ControlMode.Position, position);
  }


  public void moveToTransform(Transform2d transfrom) {
    xAxisMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getX()));
    yAxisMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getY()));
  }

  /** Runs the intake on the arm */
  public Command runIntake() {
    return run(() -> ToggleIntakeRunning(currentLimit));
  }

  /** Centering Command */
  public Command centerOnTarget(Transform2d robotToTarget) {
    return run(() -> moveToTransform(robotToTarget));
  }
  public Command percentCommand(double percent) {
    return run( ()-> xAxisMotor.set(ControlMode.PercentOutput, percent));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // #region Homing Commands
  // arm x = left -> right
  // arm y = up -> down
  public Command homeArmX() {
    return run(
    () -> xAxisMotor.set(ControlMode.PercentOutput, 0.1))
    .until(() -> xAxisforwardLimitSwitch.getAsBoolean() == true) // x has hit left limit
      .andThen(
        runOnce(()-> SmartDashboard.putNumber("X axis pos", xAxisMotor.getSelectedSensorPosition())),
        runOnce(()->xAxisMotor.setSelectedSensorPosition(0)), // zero motor on left side
        run(() -> xAxisMotor.set(ControlMode.PercentOutput, -0.1))
        .until(() -> xAxisreverseLimitSwitch.getAsBoolean() == true),
        // x has hit right limit
        runOnce(() -> xAxisMotorMax = xAxisMotor.getSelectedSensorPosition()), // get max value on the right side
        runOnce(()->SmartDashboard.putNumber("X Axis Max", xAxisMotorMax)),
        run(() -> moveXAxis(xAxisMotorMax / 2))
           );
        
        
  }

  public Command homeArmY() {
    return run(
        () -> yAxisMotor.set(ControlMode.PercentOutput, 0.1)).until(yAxisTopLimitSwitch) // arm has hit highest point
        .andThen(
            () -> yAxisMotor.setSelectedSensorPosition(0) // zero the motor at its highest point
        )
        .andThen(
            run(() -> yAxisMotor.set(ControlMode.PercentOutput, -0.1)))
        .until(yAxisBottomLimitSwitch) // arm has hit lowest point
        .andThen(
            () -> yAxisMotorMax = yAxisMotor.getSelectedSensorPosition() // get max value
        )
        .andThen(() -> moveYAxis(yAxisMotorMax / 2))
        .andThen(() -> yHomingComplete = true); // y axis homing is complete
  }

  public Command homeArm() {
    return run(this::homeArmX).andThen(this::homeArmY);
  }

  // #endregion
  // #region getters for limit switches
  public void ToggleIntakeRunning(BooleanSupplier currentlimit) {
    if (intakeRunning && currentLimit.getAsBoolean()) {
      intake.set(0.5);
      intakeRunning = true;
    } else {
      intake.set(0);
      intakeRunning = false;
    }
  }

  public boolean getForwardLimitX() {
    if (xAxisMotor.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getReverseLimitX() {
    if (xAxisMotor.isRevLimitSwitchClosed() == 1) {
      return true;
    } else if (xAxisMotor.isRevLimitSwitchClosed() == 0) {
      return false;
    }
    return false;
  }

  public boolean getForwardLimitY() {
    if (yAxisMotor.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getReverseLimitY() {
    if (yAxisMotor.isRevLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }
  // #endregion

}
