package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        public static final int Left_Front_ID = 1;
        public static final int Right_Front_ID = 3;
        public static final int Left_Back_ID = 2;
        public static final int Right_Back_ID = 4;

        // 1 motor rev = 2048 ticks
        // gearRatio motor revs = 1 wheel rev
        // 1 wheel rev = 1 wheel circumference travelled
        // 1 wheel circumference = pi*wheel diameter
        
        private static final double gearRatio = 26.0;
        private static final double wheelDiameterMeters = Units.inchesToMeters(6);
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
        private static final double fieldLength = Units.inchesToMeters((54*12) + 3.25);
        private static final double fieldWidth = Units.inchesToMeters((26*12) + 3.5);
        // TODO: measure origin to x translation
        private static final Translation2d originToX = new Translation2d(0, 0);
        // TODO: handle alliance switching
        public static final AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(
            List.of(
              new AprilTag(1, createTagPose(Units.inchesToMeters(440.625), Rotation2d.fromDegrees(63), Units.inchesToMeters(23.75))),//new Pose3d(0, 0, Units.inchesToMeters(23.75), new Rotation3d(VecBuilder.fill(0, 0, 1), 0))),
              new AprilTag(2, createTagPose(Units.inchesToMeters(370), Rotation2d.fromDegrees(123), Units.inchesToMeters(51.5))),
              new AprilTag(3, createTagPose(Units.inchesToMeters(484.625), Rotation2d.fromDegrees(91), Units.inchesToMeters(26.25))),
              new AprilTag(5, createTagPose(Units.inchesToMeters(271), Rotation2d.fromDegrees(33), Units.inchesToMeters(44))),
              new AprilTag(6, createTagPose(Units.inchesToMeters(273), Rotation2d.fromDegrees(302), Units.inchesToMeters(57.0625))),
              new AprilTag(7, createTagPose(Units.inchesToMeters(211.5), Rotation2d.fromDegrees(0), Units.inchesToMeters(43)))
            ), VisionConstants.fieldLength, VisionConstants.fieldWidth
        );

        // TODO: fill out robotToCamera transform once robot is designed
        public static final Transform3d robotToCamera = new Transform3d();

        private static Pose3d createTagPose(double distanceMeters, Rotation2d measuredAngle, double heightMeters) {
            // Step 1.start at origin
            Pose2d originPose = new Pose2d();
            // Step 2.  translate to measurement point
            // at x, facing away on long axis
            Pose2d center = originPose.transformBy(new Transform2d(originToX, new Rotation2d()));
            // Step 3. rotate tag angle to new coord system
            Rotation2d angleToTag = Rotation2d.fromDegrees((measuredAngle.getDegrees() - 90) * -1);
            // Step 4. find and apply translation to tag (on ground)
            Pose2d tagPose = center.transformBy(new Transform2d(new Translation2d(distanceMeters, angleToTag), new Rotation2d()));
            // Step 5. move pose up to tag height
            Pose3d tagPose3d = new Pose3d(tagPose.getX(), tagPose.getY(), heightMeters, new Rotation3d());
            // Done!
            return tagPose3d;
        }
    }
}
