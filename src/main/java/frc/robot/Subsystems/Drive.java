// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

//#region imports
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Simulation.DriveSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
//#endregion
public class Drive extends SubsystemBase implements Loggable {
  /** Creates a new Drive. */
  //#region declarations
  public  WPI_TalonFX Left_Front = new WPI_TalonFX(DriveConstants.Left_Front_ID);
  WPI_TalonFX Right_Front = new WPI_TalonFX(DriveConstants.Right_Front_ID);
  WPI_TalonFX Left_Back = new WPI_TalonFX(DriveConstants.Left_Back_ID);
  WPI_TalonFX Right_Back = new WPI_TalonFX(DriveConstants.Right_Back_ID);
  MotorControllerGroup LeftMCG = new MotorControllerGroup(Left_Front, Left_Back);
  MotorControllerGroup RightMCG = new MotorControllerGroup(Right_Front, Right_Back);
  DifferentialDrive diffDrive = new DifferentialDrive(LeftMCG, RightMCG);

  AHRS gyro = new AHRS();

  Vision vision;

  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(DriveConstants.trackwidthMeters), new Rotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d(5, 2, Rotation2d.fromDegrees(45)));
  DoubleArrayPublisher posePub = NetworkTableInstance.getDefault().getTable("Poses").getDoubleArrayTopic("RobotPose").publish();
  DifferentialDrive.WheelSpeeds ws = new WheelSpeeds(0, 0);
  DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.Shifter_Forward_Channel, DriveConstants.Shifter_Reverse_Channel);
  public boolean isHighGear;
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  //Simulation Stuff
  DriveSim driveSim;
  //#endregion
  
  
  public Drive(Vision vision) {
    compressor.enableDigital();
    isHighGear = false;

    Left_Front.configFactoryDefault();
    Right_Front.configFactoryDefault();
    Left_Back.configFactoryDefault();
    Right_Back.configFactoryDefault();

    Left_Back.follow(Left_Front);
    Right_Back.follow(Right_Front);

    Left_Back.setInverted(InvertType.FollowMaster);
    Right_Back.setInverted(InvertType.FollowMaster);
    

    this.vision = vision;
    gyro.calibrate();

    driveSim = new DriveSim(Left_Front, Right_Front, gyro, poseEstimator);
  }

  @Log
  private double getLeftCmd() {
    return LeftMCG.get();
  }

  @Log
  private double getRightCmd() {
    return RightMCG.get();
  }

  @Log
  private double getLeftWheelSpeed() {
    return ws.left;
  }
  
  /**
   * Command to drive the robot
   * @param throttle
   * - % of total forward motor power
   * @param turn
   * - % of total turning power
   * @return
   * Command to drive the robot
   */
  public Command ArcadeDrive(DoubleSupplier throttle, DoubleSupplier turn) {
    return run(()-> {
      diffDrive.arcadeDrive(throttle.getAsDouble(), turn.getAsDouble());
    });
  }
  /**
   * @return
   * command that shifts the gears on the robot
   */
  public Command Shift() {
    return runOnce(() -> {
      if (isHighGear) {
        shifter.set(Value.kOff);
        isHighGear = false;
       } else {
        shifter.set(Value.kForward);
        isHighGear = true;
       }
       SmartDashboard.putBoolean("Gear", isHighGear);
    });
  }
  /**
   * gets the distance that the right side of the robot traveled in meters 
   * @return
   * the distance the right side of the robot has traveled
   */
  private double getLeftDistanceMeters() {
    return Left_Front.getSelectedSensorPosition() * DriveConstants.distancePerTickMeters;
  }
  /**
   * gets the distance that the right side of the robot traveled in meters 
   * @return
   * the distance the right side of the robot has traveled
   */
  private double getRightDistanceMeters() {
    return Right_Front.getSelectedSensorPosition() * DriveConstants.distancePerTickMeters;
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  @Override
  public void periodic() {
    poseEstimator.update(gyro.getRotation2d(), getRightDistanceMeters(), getLeftDistanceMeters());
    //vision.setReferencePose(poseEstimator.getEstimatedPosition());
    // if we see targets
    if (vision.getCurrentPoseEstimate().isPresent()) {
      // if the pose is reasonably close
      if (vision.getCurrentPoseEstimate().get().estimatedPose.toPose2d().getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) < 1.5) {
        //poseEstimator.addVisionMeasurement(vision.getCurrentPoseEstimate().get().estimatedPose.toPose2d(), vision.getCurrentPoseEstimate().get().timestampSeconds);
      }
    }

    posePub.set(new double[] {
      poseEstimator.getEstimatedPosition().getX(),
      poseEstimator.getEstimatedPosition().getY(),
      poseEstimator.getEstimatedPosition().getRotation().getDegrees()
    });
  }

  public void simulationPeriodic() {
    driveSim.run();
    vision.setSimPose(poseEstimator.getEstimatedPosition());
  }
  
}