// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

//#region imports
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRSSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utility.Conversions;


public class DriveSim {
    //#region declarations
    private WPI_TalonFX rightMotor, leftMotor;

    private TalonFXSimCollection leftSim, rightSim;

    private Field2d field2d = new Field2d();
    //drive odometry object
    private DifferentialDriveOdometry SimOdometry;
    //starting pose 2d (not really necessary)
    private Pose2d startPose2d = new Pose2d(5.0, 5.0, new Rotation2d(45.0));

    private AHRS gyro;
    private AHRSSim gyroSim;
Conversions conversions;
    //drivetrain sim object
    
    private DifferentialDrivetrainSim drivetrainSim  = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2),//2 Falcon 500s on each side of the robot
        DriveConstants.gearRatio,//gear ratio between the wheels and the encoder on the robot
        2.1, //MOI? of the robot (your supposed to get it from the cad model but I just made this up)
        30,//mass of the robot (I also made this up)
        Units.inchesToMeters(3),//radius of the robot's wheels
        0.546, //distance between the robot's wheels in meters
        null); //standard deviation in your measurement devices
    //#endregion

    /**
         * Constructes a Drivetrain simulation
         * @param Front_left_Motor
         * Left motor you want to simulate
         * @param Front_Right_Motor
         * Right motor you want to simulate
         * @param gyro
         * Gyro you are using
         */
    public DriveSim(WPI_TalonFX Front_left_Motor, WPI_TalonFX Front_Right_Motor, AHRS gyro){

        this.leftMotor = Front_left_Motor;
        this.rightMotor = Front_Right_Motor;
        
        this.rightSim = rightMotor.getSimCollection();
        this.leftSim = leftMotor.getSimCollection();

        this.gyro = gyro;
        this.gyroSim = new AHRSSim();

        this.SimOdometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0, startPose2d);
        
        field2d.setRobotPose(startPose2d);
    }
    /**
     * 
     * @return
     * The field 2d object displayed during simulation
     */
    public Field2d getField() {
        return field2d;
    }
    /**
     * runs the DrivetrainSimulation
     */
    public void run() {
        conversions = new Conversions();
        drivetrainSim.setInputs(-leftSim.getMotorOutputLeadVoltage(), rightSim.getMotorOutputLeadVoltage());

        drivetrainSim.update(0.02);

        leftSim.setIntegratedSensorRawPosition(
            conversions.distanceToNativeUnits(drivetrainSim.getLeftPositionMeters())
        );
        leftSim.setIntegratedSensorVelocity(
           conversions.velocityToNativeUnits(drivetrainSim.getLeftVelocityMetersPerSecond())
        );

        rightSim.setIntegratedSensorRawPosition(
            conversions.distanceToNativeUnits(drivetrainSim.getRightPositionMeters())
        );
        rightSim.setIntegratedSensorVelocity(
            conversions.velocityToNativeUnits(drivetrainSim.getRightVelocityMetersPerSecond())
        );
        gyroSim.setYaw(drivetrainSim.getHeading().getDegrees());

        SimOdometry.update(
        gyro.getRotation2d(), 
        conversions.nativeUnitsToDistanceMeters(leftMotor.getSelectedSensorPosition()), /*update distance the left side of the robot has traveled*/
        conversions.nativeUnitsToDistanceMeters(rightMotor.getSelectedSensorPosition()) /*update distance the right side of the robot has traveled*/
        );

        field2d.setRobotPose(SimOdometry.getPoseMeters()); /*update robot position on the field */

        //useful date that is put on the SmartDashboard tab in NetworkTables
        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putNumber("Odometry X", SimOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", SimOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Rotation", gyro.getRotation2d().getDegrees());
    }
    

}
