// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
//#region imports
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
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
  Arm arm;
  Intake intake;
  Conversions conversions = new Conversions(this::getGearRatio);
  @Log
  double yaw;

  // basic drive
  WPI_TalonFX left_Middle, right_Middle;
  MotorControllerGroup leftMotorControllerGroup, rightMotorControllerGroup;
  
  // Drive Control
  DifferentialDriveWheelVoltages wheelVolts;
  DifferentialDriveWheelSpeeds currentDesiredWheelSpeeds;
  DifferentialDrivePoseEstimator poseEstimator;
  LTVDifferentialDriveController ltvController;
   public DifferentialDrive differentialDrive;
  CenterOnChargeStation centerOnChargeStation;
  
  // TODO: add substation translation
  Translation2d hpStation = new Translation2d(0, 0);
  Timer trajectoryTimer = new Timer();
  
  // pneumatics
  DoubleSolenoid shifter;
  Compressor compressor;
  
 // @Log(name = "Distance to HP Station (ft.)", tabName = "Driver")
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

  public Drive(Arm arm, Intake intake) {
    this.arm = arm;
    this.intake = intake;
    centerOnChargeStation = new CenterOnChargeStation(this);
    
  //#region motors
    left_Middle = new WPI_TalonFX(4);
    right_Middle = new WPI_TalonFX(1);
    
  

    differentialDrive = new DifferentialDrive(left_Middle, right_Middle);
    wheelVolts = new DifferentialDriveWheelVoltages();
    currentDesiredWheelSpeeds = new DifferentialDriveWheelSpeeds();

    left_Middle.setNeutralMode(NeutralMode.Coast);
  
    right_Middle.setNeutralMode(NeutralMode.Coast);
  //#endregion
  
  //#region Simulation
   

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
    // ltvController =
    //     new LTVDifferentialDriveController(DriveConstants.plant, DriveConstants.trackwidthMeters,
    //     // states = x pos, y pos, angle, left speed, right speed
    //     VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(2),
    //     Units.degreesToRadians(0.5), 0.1, 0.1),
    //     // inputs = left volts, right volts
    //     VecBuilder.fill(12.0, 12.0), 0.02
    //     );
  //#endregion
   
    if (Robot.isReal()) {
      left_Middle.setInverted(true);
    } 
    yaw = gyro.getYaw();
  }

  public Command setLEDColorCommand(IntSupplier color) {
    return runOnce(()-> setledColor(color.getAsInt()));//.andThen(
    //runOnce( ()->setledColor(color.getAsInt())
    
  }
  /**
   * sets the color on the leds
   */
  public void setledColor(int color) {
    ledPWM.setRaw(color);
    // Thread.sleep(20);
    // ledPWM.setRaw(0);
  }
  /**
   * toggles the drivetrain breaks 
   */
  public Command toggleBreak() {
    return runOnce(() -> {
        if(!isHighGear){
          if (!isBreaking) {
            left_Middle.setNeutralMode(NeutralMode.Brake);
            right_Middle.setNeutralMode(NeutralMode.Brake);
          } else {
            left_Middle.setNeutralMode(NeutralMode.Coast);
            right_Middle.setNeutralMode(NeutralMode.Coast);
          }
        }
      }
    );   
  }
  /**
   * sets all the drive motors to coast mode
   */
  public Command setCoast() {
    return runOnce(()->{ 
        left_Middle.setNeutralMode(NeutralMode.Coast);
        right_Middle.setNeutralMode(NeutralMode.Coast);
      }
    );
  }
  /**
   * gets the gear the robot is currently in
   */
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
   * command that drives the robot during TeleOp
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
  /**
   * command that shifts the drivetrain gearboxes
   */
  public Command ShiftCmd() {
    return runOnce(() -> {
        if (isHighGear) {
          shifter.set(Value.kOff);
          isHighGear = false;
        } else {
          shifter.set(Value.kForward);
          isHighGear = true;
          
          left_Middle.setNeutralMode(NeutralMode.Coast);
         
          right_Middle.setNeutralMode(NeutralMode.Coast);
        }
      }
    );
  }

  public Command shiftHigh() {
    return runOnce(()-> {
      shifter.set(Value.kReverse);
    });
  }
 
  @Log
  private double getLeftDistanceTraveledMeters() {
    return -Conversions.nativeUnitsToDistanceMeters(left_Middle.getSelectedSensorPosition());
  }

  @Log
  private double getRightDistanceTraveledMeters() {
    return Conversions.nativeUnitsToDistanceMeters(right_Middle.getSelectedSensorPosition());
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
        .nativeUnitsToVelocityMetersPerSecond(left_Middle.getSelectedSensorVelocity());
  }

  @Log
  private double getRightVelocityMetersPerSecond() {
    return Conversions
        .nativeUnitsToVelocityMetersPerSecond(right_Middle.getSelectedSensorVelocity());
  }

  private double getAngularVelocityRadsPerSecond(double linearVel, double curvature) {
    return linearVel * curvature;
  }

  // public Command followTrajectory(Trajectory trajectory) {
  //   PathPlannerServer.sendActivePath(trajectory.getStates());
  //   return runOnce(() -> {
  //     poseEstimator.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getLeftDistanceTraveledMeters(),
  //         getRightDistanceTraveledMeters(), trajectory.getInitialPose());
  //     trajectoryTimer.start();
  //   }).andThen(run(() -> {
  //     field.getObject("traj").setTrajectory(trajectory);
  //     Trajectory.State trajState = trajectory.sample(trajectoryTimer.get());
  //     PathPlannerServer.sendPathFollowingData(trajState.poseMeters,
  //         poseEstimator.getEstimatedPosition());
  //     currentDesiredWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(
  //         new ChassisSpeeds(trajState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(
  //             trajState.velocityMetersPerSecond, trajState.curvatureRadPerMeter)));
  //     var nextState = trajectory.sample(trajectoryTimer.get() + 0.02);
  //     var nextWheelSpeeds = DriveConstants.kinematics.toWheelSpeeds(
  //         new ChassisSpeeds(nextState.velocityMetersPerSecond, 0, getAngularVelocityRadsPerSecond(
  //             nextState.velocityMetersPerSecond, nextState.curvatureRadPerMeter)));
  //     var ffVolts = DriveConstants.drivetrainFeedforward.calculate(
  //         currentDesiredWheelSpeeds.leftMetersPerSecond, nextWheelSpeeds.leftMetersPerSecond,
  //         currentDesiredWheelSpeeds.rightMetersPerSecond, nextWheelSpeeds.rightMetersPerSecond,
  //         0.02);
  //     var ltvVolts = ltvController.calculate(poseEstimator.getEstimatedPosition(),
  //         currentDesiredWheelSpeeds.leftMetersPerSecond,
  //         currentDesiredWheelSpeeds.rightMetersPerSecond, trajState);
  //     setWheelVoltages(ffVolts.left + ltvVolts.left, ffVolts.right + ltvVolts.right);
  //     differentialDrive.feed();
  //     // setWheelVoltages(ffVolts.left, ffVolts.right);
  //   }).until(() -> ltvController.atReference()).finallyDo((boolean end) -> {
  //     setWheelVoltages(0, 0);
  //     trajectoryTimer.stop();
  //     trajectoryTimer.reset();
  //   }));
  // }

  /**
   * drives the robot to a specific distance
   * @param distanceInches 
   * Distance you want the robot to travel in inches
   * @param isNegative 
   * are you trying to go forward to backward -
   * true = backward,
   * false = forward
   */
  boolean isNegative = false;
  public Command driveToDistance(double distanceInches, double timout) {
    return run(()-> {
        isNegative = false;
        if (distanceInches < 0) {
          isNegative =true;
        }

      if(isNegative) {
        if(left_Middle.getSelectedSensorPosition()< getEncoderCounts(distanceInches, isNegative)){
          differentialDrive.arcadeDrive(-0.7, 0.02);  
        }   
      }else {

        if(left_Middle.getSelectedSensorPosition() < getEncoderCounts(distanceInches, isNegative)){
          differentialDrive.arcadeDrive(0.7, 0);
        }
      }
    }).until(()-> left_Middle.getSelectedSensorPosition() == getEncoderCounts(distanceInches, isNegative)).withTimeout(timout);
  }

  private double getEncoderCounts(double distanceInches, boolean isNegative) {
    if (isNegative) {
      return left_Middle.getSelectedSensorPosition() - ((2048 * 26)*(distanceInches/18.84954));
    }else{
      return left_Middle.getSelectedSensorPosition() + ((2048 * 26)*(distanceInches/18.84954));
    }
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
  /**
   * gets the pitch of the robot
   * @return
   */
  public double getPitch() {
    return gyro.getRoll(); //pitch is actually roll because of how the gyro is mounted on the RIO
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }
  /**
   * turns the robot to a specific angle on the gyro
   * @param angleDegrees
   * - how many degrees do we want the gyro to turn from its current position
   * @param tolerance
   * - how percise do you want the turn to be (to tight tolerance could cause the robot to wiggle back and forth)
   */
  @Log
   double thing;
  @Log
    double setpointYaw;
  public Command turnToRelativeAngle(double angleDegrees, double tolerance) {
    final double yaw = gyro.getYaw();
    setpointYaw = yaw + angleDegrees; 
    boolean isNegative;
    
     thing = Math.abs(yaw - setpointYaw);
     //how many degrees of tolerance do we want to have in the turn

    if (setpointYaw < yaw) {
      isNegative = true;
    }else {
      isNegative =false;
    }
      if (isNegative){
        if (Math.abs(yaw-setpointYaw) > tolerance) {
          return run(()-> differentialDrive.arcadeDrive(0, -0.4))
            .until(() -> Math.abs(setpointYaw - gyro.getYaw()) < tolerance);
        } else {
          return run(()-> differentialDrive.arcadeDrive(0, 0));
        }
      } else {
        if (Math.abs(yaw-setpointYaw) >tolerance) {
          return run(()-> differentialDrive.arcadeDrive(0, 0.4))
            .until(() -> Math.abs(setpointYaw - yaw) < tolerance);
        } else {
          return run(()->differentialDrive.arcadeDrive(0, 0));
        }
      }
    }
     

  @Override
  public void periodic() {
    poseEstimator.update(
      getRotation2d(), getLeftDistanceTraveledMeters(), getRightDistanceTraveledMeters()
      );
    
    distanceToSubstation = Units.metersToFeet(
      poseEstimator.getEstimatedPosition().getTranslation().getDistance(hpStation)
      );
   
    
    // if we see targets
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
  private void runSim() {
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
