// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SysId.Logging;

/** Add your docs here. */
public class SysIdGeneralMechanismLogger extends SysIdLogger {
    private double primaryMotorVoltage = 0.0;
    
    public double getMotorVoltage() {
        return primaryMotorVoltage;
    }

    public void log(double voltage, double measuredPosition, double measuredVelocity) {
        // set desired voltage
        updateData();
        // if we have room left in the data list
        if (data.size() < dataVectorSize) {
            // add datapoints to list
            double[] dataPacket = new double[] { timestamp, voltage, measuredPosition, measuredVelocity };
            for (double d : dataPacket) {
                data.add(d);
            }
        }
        // update desired motor voltage
        primaryMotorVoltage = motorVoltage;
    }

    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
    }

    boolean isWrongMechanism() {
        return  mechanism != "Arm" && mechanism != "Elevator" && mechanism != "Simple";
    }
}
