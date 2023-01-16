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
       
        public static final int Shifter_Forward_Channel = 0;
        public static final int Shifter_Reverse_Channel = 1;

    }

    public static final class OperatorConstants {
        public static final int Driver_Controller_ID = 0; //note that controller ID's are not the same as can ID's
    }

    public static final class VisionConstants {
        // TODO: set camera name based on the actual camera name
        public static final String cameraName = "visionCam";
        private static final double fieldLength = Units.feetToMeters(54);
        private static final double fieldWidth = Units.feetToMeters(26);
       
        // TODO: fill out field layout with what we set up in the practice area
        public static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(
            
            List.of(
              new AprilTag(0, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(1, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(2, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(3, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(4, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(5, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(6, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(7, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0))),
              new AprilTag(8, new Pose3d(0, 0, 0, new Rotation3d(VecBuilder.fill(0, 0, 0), 0)))
            ), VisionConstants.fieldLength, VisionConstants.fieldWidth
        );
        // TODO: fill out robotToCamera transform once robot is designed
        public static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(), 
            new Rotation3d(0, 0, 0)
        );
        
    }
}
