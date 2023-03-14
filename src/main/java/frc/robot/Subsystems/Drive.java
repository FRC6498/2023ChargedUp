// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
//#region imports
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.Commands.CenterOnChargeStation;
import frc.robot.Utility.Conversions;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
// #endregion

public class Drive extends SubsystemBase implements Loggable {

  // #region declarations
  // Subsystems
  Vision visionSub;
  Arm arm;
  Intake intake;
  Conversions conversions = new Conversions(this::getGearRatio);

  // basic drive
  WPI_TalonFX left_Front, right_Front, left_Back, right_Back, left_Middle, right_Middle;
  MotorControllerGroup leftMotorControllerGroup, rightMotorControllerGroup;
  
  // Drive Control
  DifferentialDriveWheelVoltages wheelVolts;
  DifferentialDriveWheelSpeeds currentDesiredWheelSpeeds;
  DifferentialDrivePoseEstimator poseEstimator;
  LTVDifferentialDriveController ltvController;
   DifferentialDrive differentialDrive;
  CenterOnChargeStation centerOnChargeStation;
  
  // TODO: add substation translation
  Translation2d hpStation = new Translation2d(0, 0);
  Timer trajectoryTimer = new Timer();
  
  // pneumatics
  DoubleSolenoid shifter;
  Compressor compressor;
  
  @Log(name = "Distance to HP Station (ft.)", tabName = "Driver")
  double distanceToSubstation = -1;
  DoubleArrayPublisher posePub;
  
  // Simulation
  TalonFXSimCollection leftMotorSim, rightMotorSim;
  DifferentialDrivetrainSim drivetrainSim;
  AHRSSim gyroSim;
  @Log.Field2d
  Field2d field;

  //Misc components
  AHRS gyro;

  //LED
  static int ledcolor = 9;
  PWM ledPWM;
  
  
  //variables
  @Log.BooleanBox
  public boolean isHighGear = false;
  static boolean isBreaking = false;
//#endregion  

  public Drive(Vision vision, Arm arm, Intake intake) {
    this.visionSub = vision;
    this.arm = arm;
    this.intake = intake;
    centerOnChargeStation = new CenterOnChargeStation(this);
    
  //#region motors
    left_Front = new WPI_TalonFX(DriveConstants.left_Front_ID);
    left_Middle = new WPI_TalonFX(DriveConstants.left_Middle_ID);
    left_Back = new WPI_TalonFX(DriveConstants.left_Back_ID);
    right_Front = new WPI_TalonFX(DriveConstants.right_Front_ID);
    right_Middle = new WPI_TalonFX(DriveConstants.right_Middle_ID);
    right_Back = new WPI_TalonFX(DriveConstants.right_Back_ID);
    
    leftMotorControllerGroup = new MotorControllerGroup(left_Front, left_Back, left_Middle);
    rightMotorControllerGroup = new MotorControllerGroup(right_Front, right_Back, right_Middle);

    differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);
    wheelVolts = new DifferentialDriveWheelVoltages();
    currentDesiredWheelSpeeds = new DifferentialDriveWheelSpeeds();

    left_Front.configFactoryDefault();
    right_Front.configFactoryDefault();
    left_Back.configFactoryDefault();
    right_Back.configFactoryDefault();
    left_Front.setSelectedSensorPosition(0);
    left_Back.setSelectedSensorPosition(0);
    right_Front.setSelectedSensorPosition(0);
    right_Back.setSelectedSensorPosition(0);

    left_Back.follow(left_Front);
    right_Back.follow(right_Front);
    left_Back.setInverted(InvertType.FollowMaster);
    right_Back.setInverted(InvertType.FollowMaster);

    left_Front.setNeutralMode(NeutralMode.Coast);
    left_Back.setNeutralMode(NeutralMode.Coast);
    left_Middle.setNeutralMode(NeutralMode.Coast);
    right_Front.setNeutralMode(NeutralMode.Coast);
    right_Back.setNeutralMode(NeutralMode.Coast);
    right_Middle.setNeutralMode(NeutralMode.Coast);
  //#endregion
  
  //#region Simulation
    leftMotorSim = left_Front.getSimCollection();
    rightMotorSim = right_Front.getSimCollection();
    drivetrainSim =
     new DifferentialDrivetrainSim(DCMotor.getFalcon500(3), DriveConstants.gearRatioLow,
    2, 118, DriveConstants.wheelDiameterMeters/2.0,
     DriveConstants.trackwidthMeters, null
     );
    gyroSim = new AHRSSim();    
    field = new Field2d();
  //#endregion

  //#region Misc Components
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    shifter = 
        new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.Shifter_Forward_Channel,
        DriveConstants.Shifter_Reverse_Channel
        );
    gyro = new AHRS();
    ledPWM = new PWM(9);
    gyro.setAngleAdjustment(0);
    compressor.enableDigital();
    shifter.set(Value.kOff);
  //#endregion
  
  //#region controllers
    posePub = 
        NetworkTableInstance.getDefault().getTable("Poses").getDoubleArrayTopic("RobotPose")
        .publish();
    poseEstimator = 
        new DifferentialDrivePoseEstimator(DriveConstants.kinematics, new Rotation2d(),
        getLeftDistanceTraveledMeters(), getRightDistanceTraveledMeters(),
        new Pose2d(1, 1, Rotation2d.fromDegrees(0))
        );
    ltvController =
        new LTVDifferentialDriveController(DriveConstants.plant, DriveConstants.trackwidthMeters,
        // states = x pos, y pos, angle, left speed, right speed
        VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(2),
        Units.degreesToRadians(0.5), 0.1, 0.1),
        // inputs = left volts, right volts
        VecBuilder.fill(12.0, 12.0), 0.02
        );
  //#endregion
   
    if (Robot.isReal()) {
      leftMotorControllerGroup.setInverted(true);
    } 
  }

  public Command setLEDColorCommand(IntSupplier color) {
    return runOnce(()-> setledColor(0)).andThen(
    runOnce( ()->setledColor(color.getAsInt())).withTimeout(0.02));
  }
  public void setledColor(int color) {
    ledPWM.setRaw(color);
    // Thread.sleep(20);
    // ledPWM.setRaw(0);
  }

  public Command toggleBreak() {
    return runOnce(() -> {
        if(!isHighGear){
          if (!isBreaking) {
            left_Front.setNeutralMode(NeutralMode.Brake);
            left_Back.setNeutralMode(NeutralMode.Brake);
            left_Middle.setNeutralMode(NeutralMode.Brake);
            right_Front.setNeutralMode(NeutralMode.Brake);
            right_Back.setNeutralMode(NeutralMode.Brake);
            right_Middle.setNeutralMode(NeutralMode.Brake);
          } else {
            left_Front.setNeutralMode(NeutralMode.Coast);
            left_Back.setNeutralMode(NeutralMode.Coast);
            left_Middle.setNeutralMode(NeutralMode.Coast);
            right_Front.setNeutralMode(NeutralMode.Coast);
            right_Back.setNeutralMode(NeutralMode.Coast);
            right_Middle.setNeutralMode(NeutralMode.Coast);
          }
        }
      }
    );   
  }
  public Command setCoast() {
    return runOnce(()->{ 
        left_Front.setNeutralMode(NeutralMode.Coast);
        left_Back.setNeutralMode(NeutralMode.Coast);
        left_Middle.setNeutralMode(NeutralMode.Coast);
        right_Front.setNeutralMode(NeutralMode.Coast);
        right_Back.setNeutralMode(NeutralMode.Coast);
        right_Middle.setNeutralMode(NeutralMode.Coast);
      }
    );
  }

  public boolean getGear() {
    return isHighGear;
  }

  private double getGearRatio() {
    if (isHighGear) {
      return DriveConstants.gearRatioHigh;
    } else {
      return DriveConstants.gearRatioLow;
    }
  }

  /** 
   * @param throttle - % of total forward motor power
   * @param turn - % of total turning power
   * @return Command to drive the robot
   */
  public Command ArcadeDriveCmd(DoubleSupplier throttle, DoubleSupplier turn) {
      return run(() -> {
        differentialDrive.arcadeDrive(throttle.getAsDouble(), -turn.getAsDouble(), true);
      }
    );
  }  
 
  public Command ShiftCmd() {
    return runOnce(() -> {
        if (isHighGear) {
          shifter.set(Value.kOff);
          isHighGear = false;
        } else {
          shifter.set(Value.kForward);
          isHighGear = true;
          left_Front.setNeutralMode(NeutralMode.Coast);
          left_Back.setNeutralMode(NeutralMode.Coast);
          left_Middle.setNeutralMode(NeutralMode.Coast);
          right_Front.setNeutralMode(NeutralMode.Coast);
          right_Back.setNeutralMode(NeutralMode.Coast);
          right_Middle.setNeutralMode(NeutralMode.Coast);
        }
      }
    );
  }
 
  @Log
  private double getLeftDistanceTraveledMeters() {
    return -Conversions.nativeUnitsToDistanceMeters(left_Front.getSelectedSensorPosition());
  }

  @Log
  private double getRightDistanceTraveledMeters() {
    return Conversions.nativeUnitsToDistanceMeters(right_Front.getSelectedSensorPosition());
  }

  /*
   * @Log private double getLeftWheelSpeedFF() { return
   * currentDesiredWheelSpeeds.leftMetersPerSecond; }
   * 
   * @Log private double getRightWheelSpeedFF() { return
   * currentDesiredWheelSpeeds.rightMetersPerSecond; }
   */

  @Log
  private double getLeftVelocityMetersPerSecond() {
    return -Conversions
        .nativeUnitsToVelocityMetersPerSecond(left_Front.getSelectedSensorVelocity());
  }

  @Log
  private double getRightVelocityMetersPerSecond() {
    return Conversions
        .nativeUnitsToVelocityMetersPerSecond(right_Front.getSelectedSensorVelocity());
  }

  private double getAngularVelocityRadsPerSecond(double linearVel, double curvature) {
    return linearVel * curvature;
  }

  public Command followTrajectory(Trajectory trajectory) {
    PathPlannerServer.sendActivePath(trajectory.getStates());
    return runOnce(() -> {
      poseEstimator.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getLeftDistanceTraveledMeters(),
          getRightDistanceTraveledMeters(), trajectory.getInitialPose());
      trajectoryTimer.start();
    }).andThen(run(() -> {
      field.getObject("traj").setTrajectory(trajectory);
      Trajectory.State trajState = trajectory.sample(trajectoryTimer.get());
      PathPlannerServer.sendPathFollowingData(trajState.poseMeters,
          poseEstimator.getEstimatedPosition());
      currentDesiredWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(
          new ChassisSpeeds(trajState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(
              trajState.velocityMetersPerSecond, trajState.curvatureRadPerMeter)));
      var nextState = trajectory.sample(trajectoryTimer.get() + 0.02);
      var nextWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(
          new ChassisSpeeds(nextState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(
              nextState.velocityMetersPerSecond, nextState.curvatureRadPerMeter)));
      var ffVolts = DriveConstants.drivetrainFeedforward.calculate(
          currentDesiredWheelSpeeds.leftMetersPerSecond, nextWheelSpeeds.leftMetersPerSecond,
          currentDesiredWheelSpeeds.rightMetersPerSecond, nextWheelSpeeds.rightMetersPerSecond,
          0.02);
      var ltvVolts = ltvController.calculate(poseEstimator.getEstimatedPosition(),
          currentDesiredWheelSpeeds.leftMetersPerSecond,
          currentDesiredWheelSpeeds.rightMetersPerSecond, trajState);
      setWheelVoltages(ffVolts.left + ltvVolts.left, ffVolts.right + ltvVolts.right);
      differentialDrive.feed();
      // setWheelVoltages(ffVolts.left, ffVolts.right);
    }).until(() -> ltvController.atReference()).finallyDo((boolean end) -> {
      setWheelVoltages(0, 0);
      trajectoryTimer.stop();
      trajectoryTimer.reset();
    }));
  }
  public Command driveToDistance(double DistanceInches, boolean isNegative) {
    
    double encoderCounts = 
    left_Front.getSelectedSensorPosition() + ((2048 * 26)*(DistanceInches/18.84954));

    return run(()-> {
      if(isNegative) {
        if(left_Front.getSelectedSensorPosition()< encoderCounts){
          differentialDrive.arcadeDrive(-0.7, 0.02);
        }   
      }else {
        if(left_Front.getSelectedSensorPosition() > encoderCounts){
          differentialDrive.arcadeDrive(0.7, 0);
        }
      }
  
    }).until(()-> left_Front.getSelectedSensorPosition() == encoderCounts).withTimeout(4);
  }

  @Log
  private double getLeftVoltage() {
    return leftMotorSim.getMotorOutputLeadVoltage();
    // return wheelVolts.left;
  }

  @Log
  private double getRightVoltage() {
    return rightMotorSim.getMotorOutputLeadVoltage();
    // return wheelVolts.right;
  }

  private void setWheelVoltages(double leftVolts, double rightVolts) {
    leftMotorControllerGroup.setVoltage(leftVolts);
    rightMotorControllerGroup.setVoltage(rightVolts);
  }

  @Log
  public double getGyroAngle() {
    return getRotation2d().getDegrees();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }
  @Log
  public double get1Pitch() {
    return gyro.getRoll();
  }

  @Log
  public double getPitch() {
    return gyro.getPitch();
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    poseEstimator.update(
      getRotation2d(), getLeftDistanceTraveledMeters(), getRightDistanceTraveledMeters()
      );
    
    distanceToSubstation = Units.metersToFeet(
      poseEstimator.getEstimatedPosition().getTranslation().getDistance(hpStation)
      );
   
    visionSub.setReferencePose(poseEstimator.getEstimatedPosition());
    // if we see targets
    if (visionSub.getCurrentPoseEstimate().isPresent()) {
      // if the pose is reasonably close
      if  (
        visionSub.getCurrentPoseEstimate().get().estimatedPose.toPose2d().getTranslation()
        .getDistance(poseEstimator.getEstimatedPosition().getTranslation()) < 1.5
          ) 
        {
          EstimatedRobotPose estimatedPose = visionSub.getCurrentPoseEstimate().get();
          poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
          estimatedPose.timestampSeconds);
        }  
    }
    posePub.set(
      new double[] {poseEstimator.getEstimatedPosition().getX(),
      poseEstimator.getEstimatedPosition().getY(),
      poseEstimator.getEstimatedPosition().getRotation().getDegrees()}
    );
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field);
  }

  // #region Simulation
  public void simulationPeriodic() {
    runSim();
    // vision.setSimPose(poseEstimator.getEstimatedPosition());
  }

  /**
   * runs the DrivetrainSimulation
   */
  public void runSim() {
    drivetrainSim.setInputs(leftMotorSim.getMotorOutputLeadVoltage(),
        rightMotorSim.getMotorOutputLeadVoltage());

    drivetrainSim.update(0.02);

    leftMotorSim.setIntegratedSensorRawPosition(
        Conversions.distanceToNativeUnits(drivetrainSim.getLeftPositionMeters()));
    leftMotorSim.setIntegratedSensorVelocity(
        Conversions.velocityToNativeUnits(drivetrainSim.getLeftVelocityMetersPerSecond()));

    rightMotorSim.setIntegratedSensorRawPosition(
        Conversions.distanceToNativeUnits(drivetrainSim.getRightPositionMeters()));
    rightMotorSim.setIntegratedSensorVelocity(
        Conversions.velocityToNativeUnits(drivetrainSim.getRightVelocityMetersPerSecond()));
    gyroSim.setYaw(drivetrainSim.getHeading().getDegrees());
  }
  // #endregion
 public WaitCommand waitCommand(double timeSec) {
  return new WaitCommand(timeSec);
 }

}
