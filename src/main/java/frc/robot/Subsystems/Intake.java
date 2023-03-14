// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax intake;
  boolean intakeRunning = false;
  public Intake() {
    intake = new CANSparkMax(Constants.ArmConstants.IntakeSpark_ID, MotorType.kBrushless);
    intake.setOpenLoopRampRate(0.25);
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
      }
    );
  }
  public Command stopIntakeCmd() {
    return runOnce(() -> intake.set(0));
  }

  public Command setIntakeSpeedForward50Cmd() {
    return run(() -> intake.set(0.50));
  }

  public Command setIntakeSpeedForward100Cmd() {
    return run(() -> intake.set(1));
  }

  public Command setIntakeSpeedReverse50Cmd() {
    return run(() -> intake.set(-0.50));
  }

  public Command setIntakeSpeedReverse100Cmd() {
    return run(() -> intake.set(-1));
  }

  @Override
  public void periodic() {
  }
}
