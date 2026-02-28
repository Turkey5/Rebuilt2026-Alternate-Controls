package frc.robot;

import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Constants {
    public class ShooterConstants {
        // TODO: replace motor1canID and motor2canID with the actual values.
        public static int motor1CanId = 21;
        public static int motor2CanId = 22;
        public static double kP = 0.0005;
        public static double kI = 0.0000005;
        public static double kD = 0.0;
        public static double kFF = 5.20;
        public static double setPoint = 3000.0;
        public static double kMaxI = 100.0;
    }

    public class FeederConstants {
        public static int motor1CanId = 25;
        //public static int motor2CanId = 26;

        public static double kP = 0.0005;
        public static double kI = 0.00000005;
        public static double kD = 0.0;
        public static double kFF = 5.0;
        public static double setPoint = 4000.0;
    }

    public class RollerConstants {
        public static int rollerCanId = 27;

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kFF = 0.00015;
        public static double setPoint = 3000.0;
    }

    public class IntakeConstants {
        public static int intakeCanId = 28;

        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kFF = 20;
        public static double setPoint = 3000.0;
    }

    public class IntakeDeployerConstants {
        public static int deployerCanId = 30;
        public static double kP = 2.16;
        public static double kI = 0.0;
        public static double kD = 0.01;
        public static double upSetPoint = 0.074;
        public static double downSetPoint = 0.388;
        public static double kG = 0.16;
        public static double kV = 6.33;
        public static double kA = 0.02;
        public static double extendedLimit = 0.32;
        public static double retractLimit = 0.01;
        public static double extendSpeed = -0.1;
        public static double retractSpeed = 0.1;
    }
    public class Vision {
        private static final double distanceToFrontX = 0.38;
        private static final double distanceFromLeftY = 0.39;
        private static final double distanceFromFloorZ = 0.85;
        public static final String cameraName = "PC_Camera";
        public static final AprilTagFieldLayout tagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public static final Transform3d robotToCam = new Transform3d(distanceToFrontX, distanceFromLeftY, distanceFromFloorZ, new Rotation3d(0,0,0));
        public static final boolean useVision = true;
    }
}
