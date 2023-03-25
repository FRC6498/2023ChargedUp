// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.CenterOnChargeStation;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Intake;

/** Add your docs here. */
public class Autos extends CommandBase {

    public static Command DevPath(Drive drive, String pathName) {
        return drive.followTrajectory(PathPlanner.loadPath(pathName, DriveConstants.pathConfig));
    }
    
    public static Command balanceOnChargeStationAuto(Drive drive, Arm arm, Intake intake, CenterOnChargeStation centerOnChargeStation) {
        return arm.homeArm()
            .andThen(
                    arm.extendArmHighPID().withTimeout(3.5), 
                    intake.setIntakeSpeedForward75Cmd().withTimeout(0.75),
                    intake.stopIntakeCmd().alongWith(arm.retractArm()),
                    drive.setCoast(),
                    drive.driveToDistance(-150),
                    drive.waitCommand(1.75),
                    drive.driveToDistance(4),
                    centerOnChargeStation
            );
    }
    public static Command driveBackAuto(Drive drive, Arm arm, Intake intake) {
        return arm.homeArm()
            .andThen(
                    arm.extendArmHighPID().withTimeout(3.5),
                    intake.setIntakeSpeedForward75Cmd().withTimeout(0.75),
                    intake.stopIntakeCmd().alongWith(arm.retractArm()),
                    drive.setCoast(),
                    drive.driveToDistance(-150),
                    drive.waitCommand(1.75),
                    drive.turnToAngle(drive.getGyroAngle() - 180, 5)
                    );
    }
  
}
