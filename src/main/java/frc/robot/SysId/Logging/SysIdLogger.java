// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SysId.Logging;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.Robot;

/** Add your docs here. */
public class SysIdLogger {
    String mechanism = "";
    String testType = "";
    boolean rotate = false;
    double voltageCommand = 0.0;
    double motorVoltage = 0.0;
    double startTime = 0.0;
    double timestamp = 0.0;
    int ackNum = 0;
    final int dataVectorSize = 36000;
    ArrayList<Double> data;
    NetworkTable table;
    DoublePublisher voltageCmdPub;
    IntegerPublisher ackNumPub;
    StringPublisher testTypePub, testPub, telemetryPub;
    BooleanPublisher rotatePub, overflowPub, wrongMechPub;
    StringSubscriber testSub, testTypeSub;
    BooleanSubscriber rotateSub;
    DoubleSubscriber voltageCmdSub;
    IntegerSubscriber ackNumSub;

    public SysIdLogger() {
        System.out.println("Initializing logger");
        data = new ArrayList<>(dataVectorSize);
        LiveWindow.disableAllTelemetry();

        // set up publishers
        table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        voltageCmdPub = table.getDoubleTopic("SysIdVoltageCommand").publish();
        voltageCmdPub.set(0.0);
        ackNumPub = table.getIntegerTopic("SysIdAckNumber").publish();
        ackNumPub.set(ackNum);
        testTypePub = table.getStringTopic("SysIdTestType").publish();
        testTypePub.set(testType);
        testPub = table.getStringTopic("SysIdTest").publish();
        testPub.set("");
        rotatePub = table.getBooleanTopic("SysIdRotate").publish();
        rotatePub.set(rotate);
        overflowPub = table.getBooleanTopic("SysIdOverflow").publish();
        overflowPub.set(false);
        wrongMechPub = table.getBooleanTopic("SysIdWrongMech").publish();
        wrongMechPub.set(false);
        telemetryPub = table.getStringTopic("SysIdTelemetry").publish();
        telemetryPub.set("");

        // set up subscribers
        testSub = testPub.getTopic().subscribe("");
        testTypeSub = testTypePub.getTopic().subscribe("");
        rotateSub = rotatePub.getTopic().subscribe(false);
        voltageCmdSub = voltageCmdPub.getTopic().subscribe(0.0);
        ackNumSub = ackNumPub.getTopic().subscribe(0);
    }


    public void initLogging() {
        mechanism = testSub.get("");
        if (isWrongMechanism()) {
            wrongMechPub.set(true);
        }
        testType = testTypeSub.get("");
        rotate = rotateSub.get(false);
        voltageCommand = voltageCmdSub.get(0.0);
        startTime = Timer.getFPGATimestamp();
        data.clear();
        telemetryPub.set("");
        ackNum = (int)ackNumSub.get(0);
    }

    public double measureVoltage(List<WPI_TalonFX> controllers) {
        double sum = 0.0;
        for (int i = 0; i < controllers.size(); ++i) {
            WPI_TalonFX falcon = controllers.get(i);
            sum += falcon.getMotorOutputVoltage();
            if (Robot.isSimulation()) {
                System.out.println("Recording CTRE Voltage\n");
            }
        }
        return sum / controllers.size();
    }

    public void sendData() {
        System.out.println("Collected: " + data.size() + " data points.\n");

        overflowPub.set(data.size() >= dataVectorSize);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < data.size(); ++i) {
            sb.append(data.get(i));
            if (i < data.size() - 1) {
                sb.append(",");
            }
        }

        String type = testType == "Dynamic" ? "fast" : "slow";
        String direction = voltageCommand > 0 ? "forward" : "backward";
        String test = new StringBuilder().append(type).append("-").append(direction).toString();

        telemetryPub.set(new StringBuilder().append(test).append(";").append(sb.toString()).toString());
        ackNumPub.set(++ackNum);

        reset();
    }

    public void clearWhenRecieved() {
        if (ackNumSub.get(0) > ackNum) {
            telemetryPub.set("");
            ackNum = (int) ackNumSub.get(0);
        }
    }
    
    public void updateThreadPriority() {
        if (!Robot.isSimulation()) {
            if (!Notifier.setHALThreadPriority(true, 40) || Threads.setCurrentThreadPriority(true, 15)) {
                throw new RuntimeException("Setting RT priority failed");
            }
        }
    }

    public void updateData() {
        timestamp = Timer.getFPGATimestamp();
        // if the mechanism is supported
        if (!isWrongMechanism()) {
            // if ramp test
            if (testType == "Quasistatic") {
                // voltage = ramp rate * test elapsed time
                motorVoltage = voltageCommand * (timestamp - startTime);
            } else if (testType == "Dynamic") { // if steady state test
                // voltage = steady state voltage
                motorVoltage = voltageCommand;
            } else {
                motorVoltage = 0.0;
            }
        } else {
            motorVoltage = 0.0;
        }
    }

    public void reset() {
        motorVoltage = 0.0;
        timestamp = 0.0;
        startTime = 0.0;
        data.clear();
    }

    public void setMotorControllers(double volts, List<WPI_TalonFX> motors) {
        for (WPI_TalonFX motor : motors) {
            motor.setVoltage(volts);
        }
    }

    boolean isWrongMechanism() {
        return mechanism != "Arm" && mechanism != "Elevator" && mechanism != "Simple";
    }
}
