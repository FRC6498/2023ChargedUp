// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class Conversions {
    
    static DoubleSupplier gearRatio;

    public Conversions(DoubleSupplier gearRatio) {
      Conversions.gearRatio = gearRatio;
    }

    /**
    * converts from meters to falon500 integrated encoder ticks
    * @param positionMeters
    * position in meters
    * @return
    * meters -> ticks
    */
    public static int distanceToNativeUnits(double positionMeters){
      // meters -> wheel rotations
      double wheelRevs = positionMeters / (Math.PI * DriveConstants.wheelDiameterMeters);
      // wheel revs = motor revs / 26, so motor revs = wheel revs * 26
      double motorRevs = wheelRevs * 26.0;
      // sensor units = motor revs * 2048
      int ticks = (int)(motorRevs * 2048);

      //double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3));
      //double motorRotations = wheelRotations * gearRatio.getAsDouble();
      //int sensorCounts = (int)(motorRotations * DriveConstants.TalonFXCountsPerRev);
		  return ticks;
	  }

    /**
     * converts from velocity in m/s to encoderTicks/s
     * @param velocityMetersPerSecond
     * @return
     * velocity in m/s -> encoderTicks/s
     */
    public static int velocityToNativeUnits(double velocityMetersPerSecond) {
      double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
      double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio.getAsDouble();
      double motorRotationsPer100ms = motorRotationsPerSecond / 10;
      int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveConstants.TalonFXCountsPerRev);
      return sensorCountsPer100ms;
	  }

    public static double nativeUnitsToVelocityMetersPerSecond(double nativeUnits) {
      // encoder ticks per 100ms
      // encoder ticks per 1000ms
      nativeUnits *= 10;
      // meters per second
      return nativeUnitsToDistanceMeters(nativeUnits);
    }

    /**
     * converts from Falcon500 integrated encoder ticks to meters
     * @param sensorCounts
     * integrated encoder ticks
     * @return
     * encoder ticks -> meters
     */
    public static double nativeUnitsToDistanceMeters(double sensorCounts) {
		  if (gearRatio.getAsDouble() > 20) { // low gear
        return sensorCounts * DriveConstants.distancePerTickMetersLowGear;
      } else return sensorCounts * DriveConstants.distancePerTickMetersHighGear;
	  }

}