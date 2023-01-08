// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  PhotonCamera camera;
  RobotPoseEstimator poseEstimator;
  AprilTagFieldLayout tagLayout;
  Transform3d robotToCamera;
  Pair<Pose2d, Double> currentFieldPose;
  /** Creates a new Vision. */
  public Vision() {
    // TODO: fill out field layout with what we set up in the practice area
    tagLayout = new AprilTagFieldLayout(
      List.of(
        new AprilTag(0, null),
        new AprilTag(0, null),
        new AprilTag(0, null),
        new AprilTag(0, null),
        new AprilTag(0, null)
      ), Units.feetToMeters(54), Units.feetToMeters(26)
    );
    // TODO: fill out robotToCamera transform once robot is designed
    robotToCamera = new Transform3d(
      new Translation3d(0, null), 
      new Rotation3d(0, 0, 0)
    );
    camera = new PhotonCamera("visionCam");
    poseEstimator = new RobotPoseEstimator(tagLayout, PoseStrategy.AVERAGE_BEST_TARGETS, List.of(
      new Pair<PhotonCamera, Transform3d>(camera, robotToCamera)
    ));
  }

  public Pair<Pose2d,Double> getCurrentPoseEstimate() {
    return currentFieldPose;
  }

  @Override
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
