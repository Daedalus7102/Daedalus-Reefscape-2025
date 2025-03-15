package frc.robot;

public class Constants {
    public static final class SwerveConstants {
        //Front Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontLeft = 1;
        public static final int turnMotorIDfrontLeft = 2;
        public static final int cancoderIDfrontLeft = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 180;

        //Front Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontRight = 3;
        public static final int turnMotorIDfrontRight = 4;
        public static final int cancoderIDfrontRight = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 180;

        //Back Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackLeft = 5;
        public static final int turnMotorIDbackLeft = 6;
        public static final int cancoderIDbackLeft = 3;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 180;

        //Back Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackRight = 7;
        public static final int turnMotorIDbackRight = 8;
        public static final int cancoderIDbackRight = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 180;

        //PID values [We will assume that we have to use the same value for all 4 modules] (Used in "Chassis" class)
        public static final double genericModulekP = 0.005;
        public static final double genericModulekI = 0.0;
        public static final double genericModulekD = 0.0;

        //Information used to know the distance that the chassi has moved
        public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.25;
        public static final double driveRPS2MPS = driveRevsToMeters;

        //Variable setting the maximum speed of the modules (Used in "Chassi" class)
        public static double chassisHighMaxOutput = 0.9;
        public static final double chassisLowMaxOutput = 0.1;

        public static final double chassisXYAccelerationkP = 0.9;
        public static final double chassisZAccelerationkP = 1.2;
        public static final double kDeadband = 0.10;

        public static final class AutoRotateConstants {
            public static final double chassisZAutoRotationkP = 0.04;
            public static final double autoRotationkDeadband = 0.30;
            public static final double autoRotationMaxOutput = 0.2;
        }

        public static final class AimbotConstants {
            public static final double leftTargetTXAimbotReef = 12.16;
            public static final double leftTargetTYAimbotReef = 12.80;

            public static final double rightTargetTXAimbotReef = -16.11;
            public static final double rightTargetTYAimbotReef = 8.55;
    
            public static final double algaeTargetTXAimbot = 0.26;
            public static final double algaeTargetTYAimbot = 10.40;

            public static final double xAprilTagThreshold = 0.2 ;
            public static final double yApriltagThreshold = 0.1;
    
            public static final double xDriveMaxSpeed = 0.5;
            public static final double yDriveMaxSpeed = 0.5;
            public static final double kPdriveX = 0.7;
            public static final double kPdriveY = 2.1;

        }
    }
    public static final class ElevatorConstants{
        public static final int elevatorLeftMotorID= 9;
        public static final int elevatorRightMotorID= 10;

        public static final int upperLimitSwitchID = 2;
        public static final int lowerLimitSwitchID = 3;

        public static final double elevatorHighkP = 0.09;
        public static final double elevatorLowKp = 0.03;
        public static final double elevatorkI = 0;
        public static final double elevatorkD = 0;

        public static final double elevatorMotorsRiseMaxOutput = 0.6;// 0.8;
        //  Lower output MUST be negative
        public static final double elevatorMotorsLowerMaxOutput = -0.6;// -0.8;

        public static final double elevatorkDeadBand = 3;

        // Elevator setpoints for coral
        public static final double HOMEPosition = 0;
        public static final double READ_REEF_APRILTAGPosition = 20; 
        public static final double L1Position = 0; //
        public static final double L2Position = 35;
        public static final double L3Position = 78;
        public static final double L4Position = 155;
        public static final double PICKUPPosition = 27;
        public static final double elevatorMaxHeight = 215;// 215; //

        // Elevator setpoints for algae
        public static final double FLOOR_INTAKE_ALGAEPosition = 10;
        public static final double PROCESSOR_EJECTPosition = 12; //
        public static final double BETWEEN_L2_AND_L3Position = 84; //
        public static final double BETWEEN_L3_AND_L4Position = 122; //
        public static final double NETPosition = 10; //
    }

    public static final class Intakes{
        public static final class AlgaeIntakeConstants{
            public static final int algaeIntakePivotMotorID = 11;
            public static final int algaeIntakeLeftMotorID = 12;
            public static final int algaeIntakeRightMotorID = 13;

            public static final int algaePivotCancoderID = 5;

            public static final double algaePivotkP = 0.02;
            public static final double algaePivotkI = 0;
            public static final double algaePivotkD = 0;    

            public static final double algaeIntakeEjectVelocity = 0.2;
            public static final double algaeIntakeIntakeVelocity = -0.2;
            public static final double algaeIntakeSecureAlgaeValocity = 0.05;

            public static final double algaePivotEncoderOffset = 0;
            public static final double algaePivotMotorMaxPositiveOutPut = 0.2;
            public static final double algaePivotMotorMaxNegativeOutput = -0.2;

            public static final double pivotMotorkDeadBand = 1;

            // Algae pivot setpoints
            public static final double FLOOR_INTAKEPosition = 167;
            public static final double PROCCESOR_EJECTPosition = 145;
            public static final double BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position = 167;
            public static final double HOMEPosition = 90.69;
            public static final double HOME_WITH_ALGAEPosition = 130;
            public static final double NET_EJECTPosition = 130;    
            // public static final double GeneralAlgaeEjectGaolPosition = 0;  
            public static final double pivotMaxAngle = 257.10;
            public static final double pivotMinAngle = 93.69;

            public enum MergedAlgaeScorePositions {
                FLOOR_INTAKE,
                PROCCESOR_EJECT,
                BETWEEN_L2_AND_L3Position,
                BETWEEN_L3_AND_L4Position,
                HOME,
                NET_EJECTPosition
            }
        }

        public static final class CoralIntakeConstants{
            public static final int coralIntakePivotMotorID = 14;
            public static final int coralIntakeLeftMotorID = 15;
            public static final int coralIntakeRightMotorID = 16;

            public static final int coralPivotCancoderID = 6;

            public static final double coralPivotkP = 0.1;
            public static final double coralPivotkI = 0;
            public static final double coralPivotkD = 0;

            public static final double coralIntakeEjectVelocity = -0.2;
            public static final double coralIntakeIntakeVleocity = 0.2;
            public static final double coralIntakeSecureCoralVelocity = 0.1;

            public static final double coralPivotEncoderOffset = -60;
            public static final double coralPivotMotorMaxPositiveOutPut = 0.8;
            public static final double coralPivotMotorMaxNegativeOutput = -0.8;

            public static final double pivotMotorkDeadBand = 0.1;

            // Coral pivot setpoints
            public static final double HOMEPosition = 59.60;
            public static final double INTAKE_PICKUPPosition = 34.83; 
            public static final double L1Position = -10; 
            public static final double L2_and_L3Position = -25;
            public static final double L4Position = -40;
            // public static final double GeneralEJECTPosition = 0;
            public static final double pivotMaxAngle = 59.70;
            public static final double pivotMinAngle = -99.30;

            public enum MergedCoralScorePositions {
                READ_REEF_APRILTAG,
                HOME,
                INTAKE,
                L1,
                L2,
                L3,
                L4
            }
        }
    }

    public static final class ClimberConstants {
        public static final int leftMotorID = 17;
        public static final int rightMotorID = 18;

        public static final double climberMaxPositiveOutput = 0.8;
        public static final double climberMaxNegativeOutput = -0.8;

        public static final double HOMEPosition = 0;
        public static final double CLIMBPosition = 50;
        public static final double climberMaxExtension = 50;
    }

    public enum ClimberMode {
        HOME,
        CLIMB
    }
}