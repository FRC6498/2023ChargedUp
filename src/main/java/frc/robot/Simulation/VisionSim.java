// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class VisionSim {
    SimVisionSystem visionSystem;
    Pose2d robotPose;
    public VisionSim() {
        visionSystem = new SimVisionSystem(
            VisionConstants.cameraName, 
            VisionConstants.camDiagFOV, 
            VisionConstants.robotToCamera, 
            0,
            VisionConstants.camResolutionWidth,
            VisionConstants.camResolutionHeight,
            VisionConstants.minTargetArea
        );

        visionSystem.addVisionTargets(VisionConstants.tagLayout);
        robotPose = new Pose2d();
    }

    public void setRobotPose(Pose2d pose) {
        robotPose = pose;
    }

    public void run() {
        visionSystem.processFrame(robotPose);
    }
}
