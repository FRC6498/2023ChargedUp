// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.VisionConstants;
import frc.robot.Simulation.VisionSim;

public class Vision {
  PhotonCamera camera;
  RobotPoseEstimator poseEstimator;
  Pair<Pose2d, Double> currentFieldPose;
  VisionSim visionSim;

  public Vision() {
    camera = new PhotonCamera(VisionConstants.cameraName);
    poseEstimator = new RobotPoseEstimator(VisionConstants.tagLayout, PoseStrategy.AVERAGE_BEST_TARGETS, List.of(
      new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.robotToCamera)
      )
    );
    visionSim = new VisionSim();
  }
  /**
   * gives you the current estimate of your field pose
   * @return
   * your current pose estimate
   */
  public Pair<Pose2d,Double> getCurrentPoseEstimate() {
    return currentFieldPose;
  }
  
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<Pair<Pose3d,Double>> poseResult = poseEstimator.update();
    if (poseResult.isPresent()) {
      currentFieldPose = new Pair<Pose2d, Double>(poseResult.get().getFirst().toPose2d(), Timer.getFPGATimestamp() - poseResult.get().getSecond());
    } else {
      currentFieldPose = new Pair<Pose2d,Double>(null, 0.0);
    }
  }
}
