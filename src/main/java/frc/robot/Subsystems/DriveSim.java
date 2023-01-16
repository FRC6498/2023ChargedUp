// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

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

/** Add your docs here. */
public class DriveSim {
    private WPI_TalonFX rightMotor, leftMotor;

    private TalonFXSimCollection leftSim, rightSim;

    private Field2d field2d = new Field2d();
    private DifferentialDriveOdometry SimOdometry;
    private Pose2d startPose2d = new Pose2d(5.0, 5.0, new Rotation2d(45.0));

    private AHRS gyro;
    private AHRSSim gyroSim;

    private DifferentialDrivetrainSim drivetrainSim  = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(
        2),
        DriveConstants.gearRatio,
        2.1, 
        30,
        Units.inchesToMeters(3),
        0.546, 
        null);
        //TODO:change Simulation gyro from pidgeon to Navx to match actuall robot
    public DriveSim(WPI_TalonFX Front_left_Motor, WPI_TalonFX Front_Right_Motor, AHRS gyro){
        leftMotor = Front_left_Motor;
        rightMotor = Front_Right_Motor;
        
        rightSim = rightMotor.getSimCollection();
        leftSim = leftMotor.getSimCollection();

        this.gyro = gyro;
        gyroSim = new AHRSSim();

        SimOdometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0, startPose2d);
        
        field2d.setRobotPose(startPose2d);
    }
    public Field2d getField() {
        return field2d;
    }
    public void run() {
        drivetrainSim.setInputs(-leftSim.getMotorOutputLeadVoltage(), rightSim.getMotorOutputLeadVoltage());

        drivetrainSim.update(0.02);

        leftSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(drivetrainSim.getLeftPositionMeters())
        );
        leftSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(drivetrainSim.getLeftVelocityMetersPerSecond())
        );

        rightSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(drivetrainSim.getRightPositionMeters())
        );
        rightSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(drivetrainSim.getRightVelocityMetersPerSecond())
        );
        gyroSim.setYaw(drivetrainSim.getHeading().getDegrees());

        SimOdometry.update(
        gyro.getRotation2d(), 
        nativeUnitsToDistanceMeters(leftMotor.getSelectedSensorPosition()),
        nativeUnitsToDistanceMeters(rightMotor.getSelectedSensorPosition())
        );

        field2d.setRobotPose(SimOdometry.getPoseMeters());
        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putNumber("Odometry X", SimOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", SimOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Rotation", gyro.getRotation2d().getDegrees());
    }

    public int distanceToNativeUnits(double positionMeters){
		double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
		double motorRotations = wheelRotations * DriveConstants.gearRatio;
		int sensorCounts = (int)(motorRotations * DriveConstants.TalonFXCountsPerRev);
		return sensorCounts;
	}
    private int velocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
		double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.gearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / 10;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveConstants.TalonFXCountsPerRev);
		return sensorCountsPer100ms;
	}

    private double nativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / DriveConstants.TalonFXCountsPerRev;
		double wheelRotations = motorRotations / DriveConstants.gearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(3));
		return positionMeters;
	}


}
