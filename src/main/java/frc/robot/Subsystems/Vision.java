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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utility.NTHelper;

public class Vision extends SubsystemBase {
  PhotonCamera camera;
  PhotonPoseEstimator poseEstimator;
  EstimatedRobotPose currentFieldPose;
  /** Creates a new Vision. */

  public Vision() {
    camera = new PhotonCamera(VisionConstants.cameraName);
    //AprilTagFieldLayout layout = new AprilTagFieldLayout(List.of(), 0, 0);
    //try {
    //  layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    //} catch (IOException ioe) {
    //  DriverStation.reportError(ioe.getMessage(), ioe.getStackTrace());
    //}
    //poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.robotToCamera);
    poseEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.robotToCamera);
    NTHelper.sendTagLayout(VisionConstants.tagLayout);
  }

  /**
   * gives you the current estimate of your field pose
   * @return
   * your current pose estimate
   */
  public EstimatedRobotPose getCurrentPoseEstimate() {
    return currentFieldPose;
  }

  public void setReferencePose(Pose2d pose) {
    poseEstimator.setReferencePose(pose);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> poseResult = poseEstimator.update();
    if (poseResult.isPresent()) {
      currentFieldPose = poseResult.get();
    } else {
      currentFieldPose = new EstimatedRobotPose(new Pose3d(), -1);
    }
  }
}
