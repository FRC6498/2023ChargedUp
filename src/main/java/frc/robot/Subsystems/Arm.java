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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utility.Conversions;


public class Arm extends SubsystemBase {
  // ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
  CANSparkMax intake;
  TalonFX xAxisMotor, yAxisMotor;
  /** Trigger for limit switch on the arm */
  Trigger xAxisforwardLimitSwitch, xAxisreverseLimitSwitch, yAxisforwardLimitSwitch, yAxisreverseLimitSwitch;

  boolean intakeRunning;
  /** Max distance the arm motors can travel in a given direction*/
  double xAxisMotorLeftMax, xAxisMotorRightMax, yAxisMotorLeftMax, yAxisMotorRightMax;

  boolean xHomingComplete, yHomingComplete;

  /** Current-based limit switch for intake motors*/
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
    xAxisreverseLimitSwitch = new Trigger(this::getForwardLimitY);
    yAxisforwardLimitSwitch = new Trigger(this::getForwardLimitY);
    yAxisreverseLimitSwitch= new Trigger(this::getReverseLimitY);


    intakeRunning = false;
  }

 //#region getters for limit switches
  public void ToggleIntakeRunning(BooleanSupplier currentlimit) {
    if (intakeRunning && currentLimit.getAsBoolean()){
      intake.set(0.5);
      intakeRunning = true;
    }else {
      intake.set(0);
      intakeRunning = false;
    }
  }

  public boolean getForwardLimitX() {
    if (xAxisMotor.isFwdLimitSwitchClosed() == 1 ) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getReverseLimitX() {
    if (xAxisMotor.isRevLimitSwitchClosed() == 1 ) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getForwardLimitY() {
    if (yAxisMotor.isFwdLimitSwitchClosed() == 1 ) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getReverseLimitY() {
    if (yAxisMotor.isRevLimitSwitchClosed() == 1 ) {
      return true;
    } else {
      return false;
    }
  }
  //#endregion

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

//#region Homing Commands
  public Command homeArmX() {
    return
    run(
        ()-> xAxisMotor.set(ControlMode.PercentOutput, 0.1)
    ).until(xAxisforwardLimitSwitch)
    .andThen(
        () -> xAxisMotorLeftMax = xAxisMotor.getSelectedSensorPosition()
    )
    .andThen(
        run(() -> xAxisMotor.set(ControlMode.PercentOutput, -0.1))
    ).until(xAxisreverseLimitSwitch)
    .andThen(
        ()-> xAxisMotorRightMax = xAxisMotor.getSelectedSensorPosition()
    )
    .andThen(()->xHomingComplete = true);
  }
  public Command homeArmY() {
    return
    run(
        ()-> yAxisMotor.set(ControlMode.PercentOutput, 0.1)
    ).until(yAxisforwardLimitSwitch)
    .andThen(
        () -> yAxisMotorLeftMax = yAxisMotor.getSelectedSensorPosition()
    )
    .andThen(
        run(() -> yAxisMotor.set(ControlMode.PercentOutput, -0.1))
    ).until(yAxisreverseLimitSwitch)
    .andThen(
        ()-> yAxisMotorRightMax = yAxisMotor.getSelectedSensorPosition()
    )
    .andThen(()->yHomingComplete = true);
  }

  public Command homeArm() {
    return run(this::homeArmX).andThen(this::homeArmX);
  }
  //#endregion


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
