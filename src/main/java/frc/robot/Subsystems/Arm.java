// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem {
  WPI_TalonFX motor;
  ArmFeedforward feedforward;
  boolean isHomed = false;
  /** Creates a new Arm. */
  public Arm() {
    super(new ProfiledPIDController(
      ArmConstants.kP, 
      0, 
      ArmConstants.kD, 
      new Constraints(
        Units.degreesToRadians(ArmConstants.maxVelocityDegreesPerSecond), 
        Units.degreesToRadians(ArmConstants.maxAccelerationDegreesPerSecondSq)
      )
    ));
    feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    motor = new WPI_TalonFX(ArmConstants.motorCanId);
    disable();
    setGoal(ArmConstants.stowed);
  }

  public Command TrackGoal(Rotation2d goal) {
    setGoal(goal);
    return new ProfiledPIDCommand(
      getController(), 
      this::getMeasurement, 
      this.getController()::getGoal, 
      this::useOutput, 
      this
    );
  }

  private void setGoal(Rotation2d goal) {
    setGoal(goal.getDegrees());
  }

  public Command home() {
    if (!isHomed) {
      return run(() -> motor.set(0.1)).until(() -> motor.isRevLimitSwitchClosed() == 1).andThen(() -> {
        isHomed = true;
        enable();
      });
    } else {
      // already homed, no-op
      return new InstantCommand(() -> {});
    }
    
  }

  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    double ffOut = feedforward.calculate(setpoint.position, setpoint.velocity);
    motor.setVoltage(output + ffOut);
  }

  @Override
  protected double getMeasurement() {
    return sensorUnitsToArmPosition(motor.getSelectedSensorPosition());
  }

  private double sensorUnitsToArmPosition(double sensorPosition) {
    // sensor units to motor rotations
    // sensors untits are bigger number so we divide
    double motorRots = sensorPosition / 2048.0;
    // motor rots to arm rots
    // 100 motor rots = 1 arm rot
    // getting smaller so divide again
    return motorRots / 100;
  }
}
