// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  WPI_TalonFX intake;
  boolean intakeRunning = false;
  
  public Intake() {
    intake = new WPI_TalonFX(Constants.ArmConstants.IntakeTalon_ID);
    intake.setNeutralMode(NeutralMode.Brake);
    intake.configOpenloopRamp(0.25);
    intake.config_kP(0,0.15);
    intake.config_kD(0, 0.4);
  }

  public boolean getCurrentLimit() {
     if (intake.getStatorCurrent() > 0.25) {
      return true;
    }else {
      return false;
    }
  }
  /** creates a wait Command  */
  public WaitCommand waitCommand(double timeSec) {
    return new WaitCommand(timeSec);
  }
  /** holds the intake at its current position (likes to stall the motor though) */
  public Command HoldIntake() {
    return stopIntakeCmd()
    .andThen(
      waitCommand(1),
      runOnce(()->intake.setSelectedSensorPosition(0)),
      run(()-> intake.set(ControlMode.Position, 0))
    );
  }
  /** stops the intake */
  public Command stopIntakeCmd() {
    return runOnce(() -> intake.set(0));
  }
  /** runs the intake at 50% forward power */
  public Command setIntakeSpeedForward50Cmd() {
    return run(() -> intake.set(0.40)).until(()-> getCurrentLimit() == true);
  }
  /** runs the intake at 75% forward power */
  public Command setIntakeSpeedForward75Cmd() {
    return run(() -> intake.set(0.75));
  }
  /** runs the intake at 50% reverse power */
  public Command setIntakeSpeedReverse50Cmd() {
    return run(() -> intake.set(-0.40));
  }
  /** runs the intake at 100% reverse power */
  public Command setIntakeSpeedReverse100Cmd() {
    return run(() -> intake.set(-0.75));
  }

  @Override
  public void periodic() {
  }
}
