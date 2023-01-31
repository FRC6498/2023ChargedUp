package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


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
        
        public static final double gearRatio = 26.0;
        public static final double wheelDiameterMeters = Units.inchesToMeters(6);
        
        public static final double distancePerTickMeters = 2048.0 * gearRatio * Math.PI * wheelDiameterMeters;
        public static final double trackwidthMeters = 1.0;
       
        public static final int Shifter_Forward_Channel = 1;
        public static final int Shifter_Reverse_Channel = 1;

    }

    public static final class OperatorConstants {
        public static final int Driver_Controller_ID = 0; //note that controller ID's are not the same as can ID's
    }

    public static final class VisionConstants {
        // TODO: set camera name based on the actual camera name
        public static final String cameraName = "visionCam";

        private static final double fieldLength = Units.inchesToMeters((54*12) + 3.25);
        private static final double fieldWidth = Units.inchesToMeters((26*12) + 3.5);
        
        // TODO: handle alliance switching

        static double archeryWallY = Units.inchesToMeters(351);
        static double bleacherY = Units.inchesToMeters(97);
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
            ), VisionConstants.fieldLength, VisionConstants.fieldWidth
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
        public static final int ArmTalonID = 5;
        public static final int ArmGearRatio = 100;
    }
    public static final class CowCatcherConstants{
        public static final int pushcatcherForwardID = 1;
        public static final int pushcatcherReverseID = 2;
    }
}
