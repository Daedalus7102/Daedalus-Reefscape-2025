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

public class Elevator extends SubsystemBase{
    // Left motor MUST be inverted because of how the robot is configured
    private final SparkMax elevatorLeftMotor = new SparkMax(ElevatorConstants.elevatorLeftMotorID, MotorType.kBrushless);
    // Right motor MUST NOT be inverted
    private final SparkMax elevatorRightMotor = new SparkMax(ElevatorConstants.elevatorRightMotorID, MotorType.kBrushless);

    DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.upperLimitSwitchID);
    DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitSwitchID);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private double goal;
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

    public enum ElevatorPosition{
        HOME,
        L1,
        L2,
        L3,
        L4,
        PICKUP
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

    public void moveElevator(ElevatorPosition elevatorPosition){
        switch (elevatorPosition) {
            case HOME:
                goal = ElevatorConstants.HomeGoalPosition;
                goalElevatorPosition = "Elevator Home";
                break;
            case L1:
                goal = ElevatorConstants.L1GoalPosition;
                goalElevatorPosition = "Elevator L1";
                break;
            case L2:
                goal = ElevatorConstants.L2GoalPosition;
                goalElevatorPosition = "Elevator L2";
                break;
            case L3:
                goal = ElevatorConstants.L3GoalPosition;
                goalElevatorPosition = "Elevator L3";
                break;
            case L4:
                goal = ElevatorConstants.L4GoalPosition;
                goalElevatorPosition = "Elevator L4";
                break;
            case PICKUP:
                goal = ElevatorConstants.PickUpGoalPosition;
                goalElevatorPosition = "Elevator PickUp";
                break;
            }

        goal = (goal > 0) ? goal : 0;
        goal = (goal < ElevatorConstants.elevatorMaxHeight) ? goal : ElevatorConstants.elevatorMaxHeight;

        PIDvalue = elevatorPID.calculate(getElevatorPosition(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveELevatorMotors(PIDvalue);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Lower limit switch", getUpperLimitSwitch());
        SmartDashboard.putBoolean("Upper limit switch", getLowerLimitSwitch());
        SmartDashboard.putNumber("Elevator position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator velocity", getElevatorVelocity());
        SmartDashboard.putNumber("Elevator PID", PIDvalue);
        SmartDashboard.putNumber("Elevator desired position", goal);
        SmartDashboard.putString("String elevator goal", goalElevatorPosition);
    }
}