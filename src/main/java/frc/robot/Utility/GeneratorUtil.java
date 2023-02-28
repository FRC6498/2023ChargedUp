// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class GeneratorUtil {
    // assumes blue alliance
    public static Trajectory generateTrajectory(Pose2d startPose, List<Translation2d> waypoints,
            Rotation2d endHeading) {
        Pose2d poseChain = startPose;
        for (Translation2d tr : waypoints) {
            poseChain.transformBy(new Transform2d(tr, new Rotation2d()));
        }
        Pose2d endPose = new Pose2d(poseChain.getTranslation(), endHeading);
        return TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose,
                DriveConstants.trajectoryConfig);
    }
}
