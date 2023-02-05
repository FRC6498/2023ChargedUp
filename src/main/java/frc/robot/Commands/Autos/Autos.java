// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Drive;

/** Add your docs here. */
public class Autos {

    public static Command Forward3Meters(Drive drive) {
        return drive.followTrajectory(
            PathPlanner.generatePath(DriveConstants.pathConfig, List.of(
                new PathPoint(drive.getPose2d().getTranslation(), drive.getPose2d().getRotation()),
                new PathPoint(drive.getPose2d().getTranslation().plus(new Translation2d(3, 0)), Rotation2d.fromDegrees(0))
            ))
        );
    }

    public static Command QuarterTurnRadius2Meters(Drive drive) {
        return drive.followTrajectory(
            TrajectoryGenerator.generateTrajectory(
                drive.getPose2d(), 
                List.of(), 
                drive.getPose2d().transformBy(new Transform2d(new Translation2d(2, 2), Rotation2d.fromDegrees(90))), 
                DriveConstants.trajectoryConfig
            )
        );
    }

    public static Command DevPath(Drive drive, String pathName) {
        return drive.followTrajectory(PathPlanner.loadPath(pathName, DriveConstants.pathConfig));
    }
}
