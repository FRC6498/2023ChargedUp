package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class Constants {
    public static final class DriveConstants {
        
        public static final int Left_Front_ID = 1;
        public static final int Right_Front_ID = 3;
        public static final int Left_Back_ID = 2;
        public static final int Right_Back_ID = 4;

        public static final int TalonFXCountsPerRev = 2048;
        
        // 1 motor rev = 2048 ticks
        // gearRatio motor revs = 1 wheel rev
        // 1 wheel rev = 1 wheel circumference travelled
        // 1 wheel circumference = pi*wheel diameter
        
        public static final double gearRatioLow = 26.0;
        public static final double gearRatioHigh = 10.71;
        // 0.1524
        public static final double wheelDiameterMeters = Units.inchesToMeters(6);
        
        public static final double distancePerTickMetersLowGear = (Math.PI * wheelDiameterMeters) / (2048 * gearRatioLow);
        public static final double distancePerTickMetersHighGear = (Math.PI * wheelDiameterMeters) / (2048 * gearRatioHigh);
        public static final double trackwidthMeters = Units.inchesToMeters(28.5);       
        public static final int Shifter_Forward_Channel = 0;
        public static final int Shifter_Reverse_Channel = 0;
        public static final double kVLinear = 0.69821;//5.7454;
        public static final double kALinear = 0.052306;
        public static final double kVAngular = 0.71675;//5.6756;
        public static final double kAAngular = 0.0253352337;
        public static final DifferentialDriveFeedforward drivetrainFeedforward = new DifferentialDriveFeedforward(kVLinear, kALinear, kVAngular, kAAngular);
        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackwidthMeters);
        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2.4, 1.5).setKinematics(kinematics);//.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(0, 0, 0), kinematics, 12));
        public static final PathConstraints pathConfig = new PathConstraints(2.4, 2);
        public static final LinearSystem<N2,N2,N2> plant = //LinearSystemId.createDrivetrainVelocitySystem(DCMotor.getFalcon500(2), 70, wheelDiameterMeters/2, trackwidthMeters/2, 5, DriveConstants.gearRatioLow);
        LinearSystemId.identifyDrivetrainSystem(
            kVLinear, 
            kALinear, 
            kVAngular, 
            kAAngular
        );
    }

    public static final class OperatorConstants {
        public static final int Driver_Controller_ID = 0; //note that controller ID's are not the same as can ID's
    } 
    public static final class VisionConstants {
        // TODO: set camera name based on the actual camera name
        public static final String cameraName = "USB_webcam";
        
        // TODO: handle alliance switching (mirror trajectories)

        static double archeryWallY = Units.inchesToMeters(351);
        static double bleacherY = Units.inchesToMeters(97);
        private static final double fieldLength = Units.inchesToMeters((54*12) + 3.25);
        private static final double fieldWidth = Units.inchesToMeters((26*12) + 3.5);
        public static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(
            List.of(
                // X axis = long axis, y axis = short axis
                // only need to measure X 2-3 times
                // Y needed for all tags
                // origin at near-right corner
                new AprilTag(1, new Pose3d(Units.inchesToMeters(120.25+300), archeryWallY, Units.inchesToMeters(23.75), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(270)))),
                //new AprilTag(2, new Pose3d(0, 0, Units.inchesToMeters(51.5),    new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(90)))),
                new AprilTag(3, new Pose3d(Units.inchesToMeters(214.375+300), Units.inchesToMeters(29.25), Units.inchesToMeters(26.25), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(180)))),
                new AprilTag(5, new Pose3d(Units.inchesToMeters(208.125), archeryWallY, Units.inchesToMeters(44), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(270)))),
                //new AprilTag(6, new Pose3d(0, 0, Units.inchesToMeters(57.0625), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(90)))),
                new AprilTag(7, new Pose3d(Units.inchesToMeters(30.5), archeryWallY, Units.inchesToMeters(43), new Rotation3d(VecBuilder.fill(0, 0, 1), Units.degreesToRadians(270))))
            ), fieldLength, fieldWidth
        );

        // TODO: fill out robotToCamera transform once robot is designed

        public static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(13.125), 0, Units.inchesToMeters(6.125)), 
            new Rotation3d(0, 0, 0)
        );
        
        public static final double camDiagFOV = 95.0;
        public static final double camPitch = robotToCamera.getRotation().getX();
        public static final double camHeight = robotToCamera.getTranslation().getZ();
        public static final double maxLEDRange = 0;
        public static final int camResolutionWidth = 1280;
        public static final int camResolutionHeight = 720;
        public static final double minTargetArea = 10;
    }
    public static final class ArmConstants{
        public static final int ArmGearRatio = 100;
        public static final int IntakeSpark_ID = 6;
        public static final int ArmPDHPortID = 1;
        public static final int xAxisMotorID = 5;
        public static final int yAxisMotorID = 8;
        public static final PowerDistribution pdh = new PowerDistribution(0, ModuleType.kRev);
    }
    public static final class CowCatcherConstants{
        public static final int pushcatcherFullForwardID = 1;
        public static final int pushcatcherFullReverseID = 1;
        public static final int pushcatcherHalfForwardID = 2;
        public static final int pushcatcherHalfReverseID = 2;
    }
}
