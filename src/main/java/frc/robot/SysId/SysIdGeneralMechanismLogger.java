// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SysId;

/** Add your docs here. */
public class SysIdGeneralMechanismLogger extends SysIdLogger {
    private double primaryMotorVoltage = 0.0;
    
    double getMotorVoltage() {
        return primaryMotorVoltage;
    }

    void log(double voltage, double measuredPosition, double measuredVelocity) {
        updateData();
        if (data.size() < dataVectorSize) {
            double[] dataPacket = new double[] { timestamp, voltage, measuredPosition, measuredVelocity };
            for (double d : dataPacket) {
                data.add(d);
            }
        }
        primaryMotorVoltage = motorVoltage;
    }

    void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
    }

    boolean isWrongMechanism() {
        return  mechanism != "Arm" && mechanism != "Elevator" && mechanism != "Simple";
    }
}
