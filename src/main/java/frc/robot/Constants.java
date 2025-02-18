package frc.robot;

public class Constants {
    public static final class SwerveConstants {
        //Front Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontLeft = 1;
        public static final int turnMotorIDfrontLeft = 2;
        public static final int cancoderIDfrontLeft = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0;

        //Front Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontRight = 3;
        public static final int turnMotorIDfrontRight = 4;
        public static final int cancoderIDfrontRight = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0;

        //Back Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackLeft = 5;
        public static final int turnMotorIDbackLeft = 6;
        public static final int cancoderIDbackLeft = 3;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0;

        //Back Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackRight = 7;
        public static final int turnMotorIDbackRight = 8;
        public static final int cancoderIDbackRight = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0;

        //PID values [We will assume that we have to use the same value for all 4 modules] (Used in "Chassis" class)
        public static final double genericModulekP = 0.005;
        public static final double genericModulekI = 0.0;
        public static final double genericModulekD = 0.0;

        //Information used to know the distance that the chassi has moved
        public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.25;
        public static final double driveRPS2MPS = driveRevsToMeters;

        //Variable setting the maximum speed of the modules (Used in "Chassi" class)
        public static final double chassisHighMaxOutput = 0.9;
        public static final double chassisLowMaxOutput = 0.2;

        public static final double chassisXYAccelerationkP = 0.9;
        public static final double chassisZAccelerationkP = 1.2;
        public static final double kDeadband = 0.10;

        public static final class AutoRotateConstants {
            public static final double chassisZAutoRotationkP = 0.04;
            public static final double autoRotationkDeadband = 0.30;
            public static final double autoRotationMaxOutput = 0.2;
        }
    }
    public static final class ElevatorConstants{
        public static final int elevatorLeftMotorID= 9;
        public static final int elevatorRightMotorID= 10;

        public static final int upperLimitSwitchID = 1;
        public static final int lowerLimitSwitchID = 0;

        public static final double elevatorkP = 0.01;
        public static final double elevatorkI = 0;
        public static final double elevatorkD = 0;

        public static final double elevatorMotorsRiseMaxOutput = 0.7;
        public static final double elevatorMotorsLowerMaxOutput = -0.5;

        public static final double HomeGoalPosition = 0;
        public static final double L1GoalPosition = 215; //
        public static final double L2GoalPosition = 130; //
        public static final double L3GoalPosition = 135; //
        public static final double L4GoalPosition = 140; //
        public static final double PickUpGoalPosition = 145; //
        public static final double elevatorMaxHeight = 110; //
    }

    public static final class Intakes{
        public static final class CoralIntakeConstants{
            public static final double coralIntakeEjectVelocity = -0.4;
            public static final double coralIntakeIntakeVleocity = 0.2;
        }

        public static final class AlgaeIntakeConstants{
            public static final double algaeIntakeEjectVelocity = -0.1;
            public static final double algaeIntakeIntakeVelocity = 0.1;
        }
    }
}