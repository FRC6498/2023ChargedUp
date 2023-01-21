// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.Simulation.VisionSim;

public class Vision {
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private Optional<EstimatedRobotPose> currentFieldPose;
  private VisionSim visionSim;

  public Vision() {
    camera = new PhotonCamera(VisionConstants.cameraName);
    poseEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.robotToCamera);
    visionSim = new VisionSim();
    currentFieldPose = Optional.empty();
  }

  /**
   * gives you the current estimate of your field pose
   * @return
   * your current pose estimate
   */

  public Optional<EstimatedRobotPose> getCurrentPoseEstimate() {
    return currentFieldPose;
  }

  public void setSimPose(Pose2d pose) {
    visionSim.setRobotPose(pose);
  }

  public void setReferencePose(Pose2d pose) {
    poseEstimator.setReferencePose(pose);
  }
  
  public void run() {
    Optional<EstimatedRobotPose> poseResult = poseEstimator.update();
    if (poseResult.isPresent()) {
      currentFieldPose = poseResult;
    } else {
      currentFieldPose = Optional.empty();
    }
    if (Robot.isSimulation()) {
      visionSim.run();
    }
  }
}
