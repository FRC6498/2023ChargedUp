// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class Conversions {
    
    DoubleSupplier gearRatio;

    public Conversions(DoubleSupplier gearRatio) {
      this.gearRatio = gearRatio;
    }

    /**
    * converts from meters to falon500 integrated encoder ticks
    * @param positionMeters
    * position in meters
    * @return
    * meters -> ticks
    */
    public int distanceToNativeUnits(double positionMeters){
      double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
      double motorRotations = wheelRotations * gearRatio.getAsDouble();
      int sensorCounts = (int)(motorRotations * DriveConstants.TalonFXCountsPerRev);
		  return sensorCounts;
	  }

    /**
     * converts from velocity in m/s to encoderTicks/s
     * @param velocityMetersPerSecond
     * @return
     * velocity in m/s -> encoderTicks/s
     */
    public int velocityToNativeUnits(double velocityMetersPerSecond) {
      double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
      double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio.getAsDouble();
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
    public double nativeUnitsToDistanceMeters(double sensorCounts) {
		  if (gearRatio.getAsDouble() > 20) { // low gear
        return sensorCounts * DriveConstants.distancePerTickMetersLowGear;
      } else return sensorCounts * DriveConstants.distancePerTickMetersHighGear;
	  }

}
