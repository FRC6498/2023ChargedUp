// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

//#region imports
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Simulation.DriveSim;
import frc.robot.Utility.Conversions;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
//#endregion

public class Drive extends SubsystemBase implements Loggable {
  /** Creates a new Drive. */
  //#region declarations
  WPI_TalonFX Left_Front = new WPI_TalonFX(DriveConstants.Left_Front_ID);
  WPI_TalonFX Right_Front = new WPI_TalonFX(DriveConstants.Right_Front_ID);
  WPI_TalonFX Left_Back = new WPI_TalonFX(DriveConstants.Left_Back_ID);
  WPI_TalonFX Right_Back = new WPI_TalonFX(DriveConstants.Right_Back_ID);
  MotorControllerGroup LeftMCG = new MotorControllerGroup(Left_Front, Left_Back);
  MotorControllerGroup RightMCG = new MotorControllerGroup(Right_Front, Right_Back);
  DifferentialDrive diffDrive = new DifferentialDrive(LeftMCG, RightMCG);

  AHRS gyro = new AHRS();

  Vision vision;

  @Log.Field2d
  Field2d field;
  Conversions conversions = new Conversions(this::getGearRatio);
  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(DriveConstants.trackwidthMeters), new Rotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  DoubleArrayPublisher posePub = NetworkTableInstance.getDefault().getTable("Poses").getDoubleArrayTopic("RobotPose").publish();
  LTVDifferentialDriveController ltv;
  Timer trajectoryTimer = new Timer();
  DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.Shifter_Forward_Channel, DriveConstants.Shifter_Reverse_Channel);
  @Log
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
    Left_Front.setSelectedSensorPosition(0);
    Left_Back.setSelectedSensorPosition(0);
    Right_Front.setSelectedSensorPosition(0);
    Right_Back.setSelectedSensorPosition(0);

    Left_Back.follow(Left_Front);
    Right_Back.follow(Right_Front);

    Left_Back.setInverted(InvertType.FollowMaster);
    Right_Back.setInverted(InvertType.FollowMaster);
    if (Robot.isReal()) {
      LeftMCG.setInverted(true);
    }
    

    this.vision = vision;
    gyro.calibrate();

    driveSim = new DriveSim(Left_Front, Right_Front, gyro, poseEstimator, this::getGearRatio);
    field = new Field2d();
    for (AprilTag tag : VisionConstants.tagLayout.getTags()) {
      field.getObject("AprilTag_" + tag.ID).setPose(tag.pose.toPose2d());
    }
    ltv = new LTVDifferentialDriveController(
      LinearSystemId.identifyDrivetrainSystem(
        0.00052047, 
        0.00036209, 
        0.00051414, 
        7.7103E-05, 
        DriveConstants.trackwidthMeters
      ), 
      DriveConstants.trackwidthMeters,
      // states = x pos, y pos, angle, left speed, right speed
      VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(2), Units.degreesToRadians(5), 0.5, 0.5),
      // inputs = left volts, right volts 
      VecBuilder.fill(12.0, 12.0),
      0.02
    );
  }

  public boolean getGear() {
    return isHighGear;
  }

  private double getGearRatio() {
    if (isHighGear) {
      return DriveConstants.gearRatioHigh;
    } else return DriveConstants.gearRatioLow;
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
    });
  }
  /**
   * gets the distance that the right side of the robot traveled in meters 
   * @return
   * the distance the right side of the robot has traveled
   */
  @Log
  private double getLeftDistanceMeters() {
    return conversions.nativeUnitsToDistanceMeters(Left_Front.getSelectedSensorPosition());
  }
  /**
   * gets the distance that the right side of the robot traveled in meters 
   * @return
   * the distance the right side of the robot has traveled
   */
  @Log
  private double getRightDistanceMeters() {
    return conversions.nativeUnitsToDistanceMeters(Right_Front.getSelectedSensorPosition());
  }

  private double getLeftVelocityMetersPerSecond() {
    return conversions.nativeUnitsToVelocityMetersPerSecond(Left_Front.getSelectedSensorVelocity());
  }

  private double getRightVelocityMetersPerSecond() {
    return conversions.nativeUnitsToVelocityMetersPerSecond(Right_Front.getSelectedSensorVelocity());
  }

  public Command followTrajectory(Trajectory trajectory) {
    return runOnce(trajectoryTimer::start).andThen(
      run(() -> 
        setWheelVoltages(
          ltv.calculate(
            poseEstimator.getEstimatedPosition(), 
            getLeftVelocityMetersPerSecond(), 
            getRightVelocityMetersPerSecond(), 
            trajectory.sample(trajectoryTimer.get())
          )
        )
      ).until(() -> trajectoryTimer.get() >= trajectory.getTotalTimeSeconds())
      .andThen(() -> setWheelVoltages(new DifferentialDriveWheelVoltages(0, 0)))
      .andThen(trajectoryTimer::stop).andThen(trajectoryTimer::reset)
    );
  }

  private void setWheelVoltages(DifferentialDriveWheelVoltages volts) {
    LeftMCG.setVoltage(volts.left);
    RightMCG.setVoltage(volts.right);
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      poseEstimator.update(gyro.getRotation2d(), -getLeftDistanceMeters(), getRightDistanceMeters());
    } else {
      poseEstimator.update(gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
    }
    vision.setReferencePose(poseEstimator.getEstimatedPosition());
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
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  public void simulationPeriodic() {
    driveSim.run();
    poseEstimator.resetPosition(getGyroAngle(), getLeftDistanceMeters(), getRightDistanceMeters(), driveSim.getPoseMeters());
    vision.setSimPose(poseEstimator.getEstimatedPosition());
  }
  
}