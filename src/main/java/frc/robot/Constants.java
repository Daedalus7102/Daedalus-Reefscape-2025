package frc.robot;

public class Constants {
    public static final class SwerveDriveConstants {
        //Front Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontLeft = 3;
        public static final int turnMotorIDfrontLeft = 1;
        public static final int cancoderIDfrontLeft = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 180;

        //Front Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontRight = 2;
        public static final int turnMotorIDfrontRight = 7;
        public static final int cancoderIDfrontRight = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 180;

        //Back Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackLeft = 8;
        public static final int turnMotorIDbackLeft = 5;
        public static final int cancoderIDbackLeft = 3;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 180;

        //Back Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackRight = 4;
        public static final int turnMotorIDbackRight = 6;
        public static final int cancoderIDbackRight = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 180;

        //PID values [We will assume that we have to use the same value for all 4 modules] (Used in "Chassis" class)
        public static final double genericModulekP = 0.0048;
        public static final double genericModulekI = 0.0;
        public static final double genericModulekD = 0.0;

        //Information used to know the distance that the chassi has moved
        public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.25;
        public static final double driveRPS2MPS = driveRevsToMeters;

        //Variable setting the maximum speed of the modules (Used in "Chassi" class)
        public static final double chassisMaxOutput = 0.9;

        public static final double chassisXYAccelerationkP = 0.8;
        public static final double chassisZAccelerationkP = 0.8;
        public static final double kDeadband = 0.05;
    }

}
