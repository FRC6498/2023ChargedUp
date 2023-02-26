// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
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
  double xAxisAbsolutePos = 0;
  /** Current-based limit switch for intake motors */
  BooleanSupplier currentLimit = new BooleanSupplier() {
    public boolean getAsBoolean() {
      if (ArmConstants.pdh.getCurrent(ArmConstants.ArmPDHPortID) > 5){
        return true;
      }else { 
        return false;
      }
    };
  };

  public Arm() {
    SmartDashboard.putNumber("xAxisAbsolutePos", xAxisAbsolutePos);
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

    yAxisMotor.setNeutralMode(NeutralMode.Brake);
    StatorCurrentLimitConfiguration config  = new StatorCurrentLimitConfiguration();
    config.currentLimit = 36;
    config.enable = true;
    yAxisMotor.configStatorCurrentLimit(config);
    intake.setSmartCurrentLimit(2);
    
  }

  public void moveXAxis(double position) {
    xAxisMotor.set(ControlMode.Position, position);
  }

  public void moveYAxis(double position) {
    xAxisMotor.set(ControlMode.Position, position);
  }
  public Command Stop() {
    return run(() -> yAxisMotor.set(ControlMode.PercentOutput, 0));
  }
  public Command moveY(double percent) {
    return run(()-> yAxisMotor.set(ControlMode.PercentOutput, percent));
  }


  public void moveToTransform(Transform2d transfrom) {
    xAxisMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getX()));
    yAxisMotor.set(ControlMode.Position, Conversions.distanceToNativeUnits(transfrom.getY()));
  }

  /** Runs the intake on the arm */
  public Command runIntake() {
    return runOnce(() -> ToggleIntakeRunning());
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
    () -> xAxisMotor.set(ControlMode.PercentOutput, 0.5))
    .until(() -> xAxisforwardLimitSwitch.getAsBoolean() == true) // x has hit left limit
      .andThen(
        runOnce(()-> SmartDashboard.putNumber("X axis pos", xAxisMotor.getSelectedSensorPosition())),
        runOnce(()->xAxisMotor.setSelectedSensorPosition(0)), // zero motor on left side
        run(() -> xAxisMotor.set(ControlMode.PercentOutput, -0.5))
        .until(() -> xAxisreverseLimitSwitch.getAsBoolean() == true),
        // x has hit right limit
        runOnce(() -> xAxisMotorMax = xAxisMotor.getSelectedSensorPosition()), // get max value on the right side
        runOnce(()->SmartDashboard.putNumber("X Axis Max", xAxisMotorMax)),
        run(() -> moveXAxis(xAxisMotorMax / 2)),
        runOnce(() -> xAxisAbsolutePos = xAxisMotorMax /2)
    );   
  }

  public Command homeArmY() {
    return run(() -> yAxisMotor.set(ControlMode.PercentOutput, -0.2)).until(()->currentLimit.getAsBoolean() == true)
    .andThen(Commands.print("HIT CURRENT LIMIT"),
     run(() -> yAxisMotor.set(ControlMode.PercentOutput,0)));
        // arm has hit highest point
         // y axis homing is completef
  }

  public Command homeArm() {
    return run(this::homeArmX).andThen(this::homeArmY);
  }

  // #endregion
  // #region getters for limit switches
  public void ToggleIntakeRunning() {
    if (intakeRunning == true) {
      intake.set(0);
      intakeRunning = false;
    } else {
      intake.set(-0.5);
      intakeRunning = true;
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
