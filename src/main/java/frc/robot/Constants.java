package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        public static final int Left_Front_ID = 1;
        public static final int Right_Front_ID = 2;
        public static final int Left_Back_ID = 3;
        public static final int Right_Back_ID = 4;
        // 1 motor rev = 2048 ticks
        // gearRatio motor revs = 1 wheel rev
        // 1 wheel rev = 1 wheel circumference travelled
        // 1 wheel circumference = pi*wheel diameter
        private static final double gearRatio = 26.0;
        private static final double wheelDiameterMeters = Units.inchesToMeters(6);
        public static final double distancePerTickMeters = 2048.0 * gearRatio * Math.PI * wheelDiameterMeters;
        public static final double trackwidthMeters = 1.0;
    }
    public static final class OperatorConstants {
         public static final int Driver_Controller_ID = 0; //note that controller ID's are not the same as can ID's
    }
}
