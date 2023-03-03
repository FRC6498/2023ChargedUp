// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class Vision {
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private Optional<EstimatedRobotPose> currentFieldPose;
  private SimVisionSystem visionSystem;

  public Vision() {
    camera = new PhotonCamera(VisionConstants.cameraName);
    poseEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout, PoseStrategy.MULTI_TAG_PNP,
        camera, VisionConstants.robotToCamera);
    currentFieldPose = Optional.empty();

    visionSystem = new SimVisionSystem(VisionConstants.cameraName, VisionConstants.camDiagFOV,
        VisionConstants.robotToCamera, 0, VisionConstants.camResolutionWidth,
        VisionConstants.camResolutionHeight, VisionConstants.minTargetArea);
  }

  /**
   * Returns the stored pose estimate and timestamp
   * 
   * @return latest estimated Pose3d and associated timestamp
   */
  public Optional<EstimatedRobotPose> getCurrentPoseEstimate() {
    return currentFieldPose;
  }

  /**
   * Sets the reference pose used by the PhotonPoseEstimator
   * 
   * @param pose drivetrain odometry pose
   */
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
      visionSystem.processFrame(poseEstimator.getReferencePose().toPose2d());
    }
  }
}
