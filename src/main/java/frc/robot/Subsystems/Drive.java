// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  WPI_TalonFX Left_Front = new WPI_TalonFX(DriveConstants.Left_Front_ID);
  WPI_TalonFX Right_Front = new WPI_TalonFX(DriveConstants.Right_Front_ID);
  WPI_TalonFX Left_Back = new WPI_TalonFX(DriveConstants.Left_Back_ID);
  WPI_TalonFX Right_Back = new WPI_TalonFX(DriveConstants.Right_Back_ID);

  MotorControllerGroup LeftMCG = new MotorControllerGroup(Left_Front, Left_Back);
  MotorControllerGroup RightMCG = new MotorControllerGroup(Right_Front, Right_Back);

  AHRS gyro = new AHRS();
  DifferentialDrive diffDrive = new DifferentialDrive(LeftMCG, RightMCG);

  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(DriveConstants.trackwidthMeters), new Rotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d());
  Supplier<Pair<Pose2d,Double>> visionPose;
  DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveConstants.Shifter_Forward_Channel, DriveConstants.Shifter_Reverse_Channel);

  public Drive(Supplier<Pair<Pose2d, Double>> visionPoseSupplier) {

    Left_Front.configFactoryDefault();
    Right_Front.configFactoryDefault();
    Left_Back.configFactoryDefault();
    Right_Back.configFactoryDefault();

    visionPose = visionPoseSupplier;
    gyro.calibrate();
  }
  /**
   * Controls the drive motors on the robot 
   * @param throttle
   * forward motor speed (percent)
   * @param turn
   * turning motor speed (percent)
   */
  public void ArcadeDrive(double throttle, double turn) {
    diffDrive.arcadeDrive(throttle, turn, true);
  }

  public void Shift() {
     switch (shifter.get()) {
      case kForward:
          shifter.set(Value.kReverse);
        break;

      case kReverse:
        shifter.set(Value.kForward);
        break;

      default:
      shifter.set(Value.kForward);
        break;
    }
  }

  public Command ArcadeDriveC(double throttle, double turn) {
    return Commands.run(()-> this.ArcadeDrive(throttle, turn), this);
  }
  public Command ShiftC() {
    return Commands.runOnce(() -> this.Shift(), this);
  }

  private double getLeftDistanceMeters() {
    return Left_Front.getSelectedSensorPosition() * DriveConstants.distancePerTickMeters;
  }

  private double getRightDistanceMeters() {
    return Right_Front.getSelectedSensorPosition() * DriveConstants.distancePerTickMeters;
  }

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation2d(), getRightDistanceMeters(), getLeftDistanceMeters());
    // if we see targets
    if (visionPose.get().getFirst() != null) {
      // if the pose is reasonably close
      if (visionPose.get().getFirst().getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) < 1.5) {
        poseEstimator.addVisionMeasurement(visionPose.get().getFirst(), visionPose.get().getSecond());
      }
    }

  }
}