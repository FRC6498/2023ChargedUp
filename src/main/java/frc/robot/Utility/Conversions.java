// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class Conversions {
    /**
    * converts from meters to falon500 integrated encoder ticks
    * @param positionMeters
    * position in meters
    * @return
    * meters -> ticks
    */
    public Conversions() {
      
    }
    public int distanceToNativeUnits(double positionMeters){
		double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
		double motorRotations = wheelRotations * DriveConstants.gearRatio;
		int sensorCounts = (int)(motorRotations * DriveConstants.TalonFXCountsPerRev);
		return sensorCounts;
	}
    /**
     * converts from velocity in m/s to encoderTicks/s
     * @param velocityMetersPerSecond
     * @return
     * velocity in m/s -> encoderTicks/s
     */
    public int velocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
		double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.gearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / 10;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveConstants.TalonFXCountsPerRev);
		return sensorCountsPer100ms;
	}
    /**
     * converts from Falcon500 integrated encoder ticks to meters
     * @param sensorCounts
     * integrated encoder ticks
     * @return
     * encoder ticks -> meters
     */
    public double nativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / DriveConstants.TalonFXCountsPerRev;
		double wheelRotations = motorRotations / DriveConstants.gearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(3));
		return positionMeters;
	}

}
