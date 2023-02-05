// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

//#region imports
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRSSim;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utility.Conversions;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
//#endregion

public class Drive extends SubsystemBase implements Loggable {
  /** Creates a new Drive. */
  //#region declarations
  Vision vision;

  WPI_TalonFX Left_Front = new WPI_TalonFX(DriveConstants.Left_Front_ID);
  WPI_TalonFX Right_Front = new WPI_TalonFX(DriveConstants.Right_Front_ID);
  WPI_TalonFX Left_Back = new WPI_TalonFX(DriveConstants.Left_Back_ID);
  WPI_TalonFX Right_Back = new WPI_TalonFX(DriveConstants.Right_Back_ID);
  MotorControllerGroup LeftMCG = new MotorControllerGroup(Left_Front, Left_Back);
  MotorControllerGroup RightMCG = new MotorControllerGroup(Right_Front, Right_Back);
  DifferentialDrive diffDrive;
  AHRS gyro = new AHRS();

  @Log.Field2d
  Field2d field;
  Conversions conversions = new Conversions(this::getGearRatio);
  //#region Controls/Pose/Trajectories  
  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(DriveConstants.kinematics, new Rotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), new Pose2d(1, 1, Rotation2d.fromDegrees(0)));
  DoubleArrayPublisher posePub = NetworkTableInstance.getDefault().getTable("Poses").getDoubleArrayTopic("RobotPose").publish();
  LTVDifferentialDriveController ltv;

  @Log(name = "Distance to HP Station (ft.)", tabName = "Driver")
  double distanceToSubstation = -1;
  //TODO: add substation translation
  Translation2d hpStation = new Translation2d(0, 0);
  DifferentialDriveWheelVoltages wheelVolts = new DifferentialDriveWheelVoltages();
  DifferentialDriveWheelSpeeds currentDesiredWheelSpeeds = new DifferentialDriveWheelSpeeds();
  Timer trajectoryTimer = new Timer();
  //#endregion
  DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.Shifter_Forward_Channel, DriveConstants.Shifter_Reverse_Channel);
  @Log.BooleanBox
  public boolean isHighGear;
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  //Simulation Stuff
  TalonFXSimCollection leftSim, rightSim;
  DifferentialDrivetrainSim driveSim;
  AHRSSim simgyro = new AHRSSim();
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
    if (Robot.isReal()) {
      LeftMCG.setInverted(true);
    }
    Left_Back.setInverted(InvertType.FollowMaster);
    Right_Back.setInverted(InvertType.FollowMaster);
    diffDrive = new DifferentialDrive(LeftMCG, RightMCG);

    this.vision = vision;
    gyro.setAngleAdjustment(0);
    field = new Field2d();
    ltv = new LTVDifferentialDriveController(
      DriveConstants.plant, 
      DriveConstants.trackwidthMeters,
      // states = x pos, y pos, angle, left speed, right speed
      VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(2), Units.degreesToRadians(0.5), 0.1, 0.1),
      // inputs = left volts, right volts 
      VecBuilder.fill(12.0, 12.0),
      0.02
    );
    leftSim = Left_Front.getSimCollection();
    rightSim = Right_Front.getSimCollection();
    driveSim = new DifferentialDrivetrainSim(
      DriveConstants.plant, 
      DCMotor.getFalcon500(2), 
      DriveConstants.gearRatioLow, 
      DriveConstants.trackwidthMeters, 
      DriveConstants.wheelDiameterMeters / 2.0, 
      null
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
    return -conversions.nativeUnitsToDistanceMeters(Left_Front.getSelectedSensorPosition());
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

  /*@Log
  private double getLeftWheelSpeedFF() {
    return currentDesiredWheelSpeeds.leftMetersPerSecond;
  }

  @Log
  private double getRightWheelSpeedFF() {
    return currentDesiredWheelSpeeds.rightMetersPerSecond;
  }*/

  @Log
  private double getLeftVelocityMetersPerSecond() {
    return -conversions.nativeUnitsToVelocityMetersPerSecond(Left_Front.getSelectedSensorVelocity());
  }

  @Log
  private double getRightVelocityMetersPerSecond() {
    return conversions.nativeUnitsToVelocityMetersPerSecond(Right_Front.getSelectedSensorVelocity());
  }

  private double getAngularVelocityRadsPerSecond(double linearVel, double curvature) {
    return linearVel * curvature;
  }
  public Command followTrajectory(Trajectory trajectory) {
    PathPlannerServer.sendActivePath(trajectory.getStates());
    return runOnce(
      () -> { 
        poseEstimator.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getLeftDistanceMeters(), getRightDistanceMeters(), trajectory.getInitialPose());
        trajectoryTimer.start();
      }).andThen(
        run(() -> {
          field.getObject("traj").setTrajectory(trajectory);
          Trajectory.State trajState = trajectory.sample(trajectoryTimer.get());
          PathPlannerServer.sendPathFollowingData(trajState.poseMeters, poseEstimator.getEstimatedPosition());
          currentDesiredWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(new ChassisSpeeds(trajState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(trajState.velocityMetersPerSecond, trajState.curvatureRadPerMeter)));
          var nextState = trajectory.sample(trajectoryTimer.get()+0.02);
          var nextWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(new ChassisSpeeds(nextState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(nextState.velocityMetersPerSecond, nextState.curvatureRadPerMeter)));
          var ffVolts = DriveConstants.drivetrainFeedforward.calculate( 
            currentDesiredWheelSpeeds.leftMetersPerSecond,
            nextWheelSpeeds.leftMetersPerSecond,
            currentDesiredWheelSpeeds.rightMetersPerSecond, 
            nextWheelSpeeds.rightMetersPerSecond,
            0.02
          );
          var ltvVolts = ltv.calculate(
            poseEstimator.getEstimatedPosition(), 
            currentDesiredWheelSpeeds.leftMetersPerSecond, 
            currentDesiredWheelSpeeds.rightMetersPerSecond, 
            trajState
          );
          setWheelVoltages(ffVolts.left+ltvVolts.left, ffVolts.right+ltvVolts.right);
          diffDrive.feed();
          //setWheelVoltages(ffVolts.left, ffVolts.right);
        })
        .until(() -> ltv.atReference())
        .finallyDo((boolean end ) -> {
          setWheelVoltages(0, 0);
          trajectoryTimer.stop();
          trajectoryTimer.reset();
        }
      )
    );
  }

  @Log
  private double getLeftVoltage() {
    return leftSim.getMotorOutputLeadVoltage();
    //return wheelVolts.left;
  }

  @Log
  private double getRightVoltage() {
    return rightSim.getMotorOutputLeadVoltage();
    //return wheelVolts.right;
  }

  private void setWheelVoltages(double leftVolts, double rightVolts) {
    LeftMCG.setVoltage(leftVolts);
    RightMCG.setVoltage(rightVolts);
  }

  @Log
  public double getGyroAngle() {
    return getRotation2d().getDegrees();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    poseEstimator.update(getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
    distanceToSubstation = Units.metersToFeet(poseEstimator.getEstimatedPosition().getTranslation().getDistance(hpStation));
    vision.setReferencePose(poseEstimator.getEstimatedPosition());
    // if we see targets
    if (vision.getCurrentPoseEstimate().isPresent()) {
      // if the pose is reasonably close
      if (vision.getCurrentPoseEstimate().get().estimatedPose.toPose2d().getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation()) < 1.5) {
        EstimatedRobotPose estPose = vision.getCurrentPoseEstimate().get();
        poseEstimator.addVisionMeasurement(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds);
      }
    }
    posePub.set(new double[] {
      poseEstimator.getEstimatedPosition().getX(),
      poseEstimator.getEstimatedPosition().getY(),
      poseEstimator.getEstimatedPosition().getRotation().getDegrees()
    });
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field);
  }

  //#region Simulation
  public void simulationPeriodic() {
    runSim();
    //vision.setSimPose(poseEstimator.getEstimatedPosition());
  }

  /**
     * runs the DrivetrainSimulation
     */
    public void runSim() {
      driveSim.setInputs(leftSim.getMotorOutputLeadVoltage(), rightSim.getMotorOutputLeadVoltage());
      
      driveSim.update(0.02);

      leftSim.setIntegratedSensorRawPosition(
        conversions.distanceToNativeUnits(driveSim.getLeftPositionMeters())
      );
      leftSim.setIntegratedSensorVelocity(
        conversions.velocityToNativeUnits(driveSim.getLeftVelocityMetersPerSecond())
      );

      rightSim.setIntegratedSensorRawPosition(
        conversions.distanceToNativeUnits(driveSim.getRightPositionMeters())
      );
      rightSim.setIntegratedSensorVelocity(
        conversions.velocityToNativeUnits(driveSim.getRightVelocityMetersPerSecond())
      );
      simgyro.setYaw(driveSim.getHeading().getDegrees());
  }
  //#endregion
}
