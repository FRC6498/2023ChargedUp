// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveForward3MetersAuto extends SequentialCommandGroup {
  /** Creates a new MoveForward3MetersAuto. */
  public MoveForward3MetersAuto(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      drive.followTrajectory(TrajectoryGenerator.generateTrajectory(
        List.of(
          drive.getPose2d(),
          drive.getPose2d().plus(new Transform2d(new Translation2d(3, new Rotation2d()), new Rotation2d()))
        ), DriveConstants.trajectoryConfig))
    );
  }
}
