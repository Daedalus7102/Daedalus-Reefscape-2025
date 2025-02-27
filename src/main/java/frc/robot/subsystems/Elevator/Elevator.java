package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;

public class Elevator extends SubsystemBase{
    // Left motor MUST be inverted because of how the robot is configured
    private final SparkMax elevatorLeftMotor = new SparkMax(ElevatorConstants.elevatorLeftMotorID, MotorType.kBrushless);
    // Right motor MUST NOT be inverted
    private final SparkMax elevatorRightMotor = new SparkMax(ElevatorConstants.elevatorRightMotorID, MotorType.kBrushless);

    DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.upperLimitSwitchID);
    DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitSwitchID);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private double goal = ElevatorConstants.HOMEPosition;
    private double PIDvalue;
    private String goalElevatorPosition = "Elevator initial positon";
    
    private final PIDController elevatorPID = new PIDController(ElevatorConstants.elevatorkP, ElevatorConstants.elevatorkI, ElevatorConstants.elevatorkD);

    public Elevator(){
        kBrakeGeneralConfig
            .idleMode(IdleMode.kBrake);
        kBrakeGeneralConfig.encoder
            .positionConversionFactor(1.382488846)
            .velocityConversionFactor(1.382488846);
        kBrakeGeneralConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kBrakeGeneralConfig.signals
            .primaryEncoderPositionPeriodMs(5);

        kCoastGeneralConfig
            .idleMode(IdleMode.kCoast);
        kCoastGeneralConfig.encoder
            .positionConversionFactor(0)
            .velocityConversionFactor(0);
        kCoastGeneralConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kCoastGeneralConfig.signals
            .primaryEncoderPositionPeriodMs(5);

        motorsToBrake();
    }

    public void motorsToBrake(){
        // Left motor MUST be inverted
        elevatorLeftMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Right motor MUST NOT be inverted
        elevatorRightMotor.configure(kBrakeGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void motorsToCoast(){
        elevatorLeftMotor.configure(kCoastGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorRightMotor.configure(kCoastGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stopMotors(){
        elevatorLeftMotor.set(0);
        elevatorRightMotor.set(0);
    }

    public double getElevatorPosition(){
        double elevatorPosition = (elevatorRightMotor.getEncoder().getPosition() + 
                                    elevatorRightMotor.getEncoder().getPosition()) / 2;
        return elevatorPosition;
    }

    private double getElevatorVelocity(){
        double elevatorVelocity = (elevatorRightMotor.getEncoder().getVelocity() + 
                                    elevatorRightMotor.getEncoder().getVelocity()) / 2;
        return elevatorVelocity;
    }

    private boolean getUpperLimitSwitch(){
        return !upperLimitSwitch.get();
        
    }

    private boolean getLowerLimitSwitch(){
        return !lowerLimitSwitch.get();
    }

    private double desaturatePidValue(double PID_value){
        if(PID_value > ElevatorConstants.elevatorMotorsRiseMaxOutput){
            PID_value =  ElevatorConstants.elevatorMotorsRiseMaxOutput;
        }
        else if(PID_value < ElevatorConstants.elevatorMotorsLowerMaxOutput){
            PID_value = ElevatorConstants.elevatorMotorsLowerMaxOutput;
        }
        return PID_value;
    }

    private void moveELevatorMotors(double velocity){
        elevatorLeftMotor.set(velocity);
        elevatorRightMotor.set(velocity);

        if (getLowerLimitSwitch() && PIDvalue < 0){
            stopMotors();
        }
        else if (getUpperLimitSwitch() && PIDvalue > 0){
            stopMotors();
        }

        if (getElevatorPosition() < 0 && PIDvalue < 0){
            stopMotors();
        }
    }

    public enum ElevatorHeights{
        // For coral
        HOME,
        READ_REEF_APRILTAG,
        L1,
        L2,
        L3,
        L4,
        PICKUP,

        // For algae
        FLOOR_ALGAE_INTAKE,
        PROCESSOR_EJECT,
        BETWEEN_L2_AND_L3,
        BETWEEN_L3_AND_L4,
        NET
    }

    public void moveElevator(ElevatorHeights elevatorHeights){
        switch (elevatorHeights) {
            // For coral
            case HOME:
                goal = ElevatorConstants.HOMEPosition;
                goalElevatorPosition = "Elevator Home " + ElevatorConstants.HOMEPosition;
                break;
            case READ_REEF_APRILTAG:
                goal = ElevatorConstants.READ_REEF_APRILTAGPosition;
                goalElevatorPosition = "Elevator read Reef apriltag " + ElevatorConstants.READ_REEF_APRILTAGPosition;
                break;
            case L1:
                goal = ElevatorConstants.L1Position;
                goalElevatorPosition = "Elevator L1 " + ElevatorConstants.L1Position;
                break;
            case L2:
                goal = ElevatorConstants.L2Position;
                goalElevatorPosition = "Elevator L2 " + ElevatorConstants.L2Position;
                break;
            case L3:
                goal = ElevatorConstants.L3Position;
                goalElevatorPosition = "Elevator L3 " + ElevatorConstants.L3Position;
                break;
            case L4:
                goal = ElevatorConstants.L4Position;
                goalElevatorPosition = "Elevator L4 " + ElevatorConstants.L4Position;
                break;
            case PICKUP:
                goal = ElevatorConstants.PICKUPPosition;
                goalElevatorPosition = "Elevator PickUp " + ElevatorConstants.PICKUPPosition;
                break;

            // FOr algae
            case FLOOR_ALGAE_INTAKE:
                goal = ElevatorConstants.FLOOR_INTAKE_ALGAEPosition;
                goalElevatorPosition = "Elevator floor algae intake " + ElevatorConstants.FLOOR_INTAKE_ALGAEPosition;
                break;
            case PROCESSOR_EJECT:
                goal = ElevatorConstants.PROCESSOR_EJECTPosition;
                goalElevatorPosition = "Elevator processor algae intake " + ElevatorConstants.PROCESSOR_EJECTPosition;
                break;
            case BETWEEN_L2_AND_L3:
                goal = ElevatorConstants.BETWEEN_L2_AND_L3Position;
                goalElevatorPosition = "Elevator between L2 and L3" + ElevatorConstants.BETWEEN_L2_AND_L3Position;
                break;
            case BETWEEN_L3_AND_L4:
                goal = ElevatorConstants.BETWEEN_L3_AND_L4Position;
                goalElevatorPosition = "Elevator between L3 and L4" + ElevatorConstants.BETWEEN_L3_AND_L4Position;
            case NET:
                break;
            }    
    }

    public boolean isAtTarget() {
        return getElevatorPosition() >= goal - ElevatorConstants.elevatorkDeadBand
        && getElevatorPosition() <= goal + ElevatorConstants.elevatorkDeadBand;
    }

    @Override
    public void periodic(){
        goal = (goal > 0) ? goal : 0;
        goal = (goal < ElevatorConstants.elevatorMaxHeight) ? goal : ElevatorConstants.elevatorMaxHeight;

        PIDvalue = elevatorPID.calculate(getElevatorPosition(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveELevatorMotors(PIDvalue);

        if (getElevatorPosition() > 100) {
            SwerveConstants.chassisHighMaxOutput = SwerveConstants.chassisLowMaxOutput;
        } else {
            SwerveConstants.chassisHighMaxOutput = 0.9;
        }
        SmartDashboard.putBoolean("Lower limit switch", getUpperLimitSwitch());
        SmartDashboard.putBoolean("Upper limit switch", getLowerLimitSwitch());
        SmartDashboard.putNumber("Elevator position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator velocity", getElevatorVelocity());
        SmartDashboard.putNumber("Elevator PID", PIDvalue);
        SmartDashboard.putNumber("Elevator desired position", goal);
        SmartDashboard.putString("String elevator goal", goalElevatorPosition);
        SmartDashboard.putBoolean("isFinished", isAtTarget());
    }
}