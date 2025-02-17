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
    
    private final PIDController elevatorPID = new PIDController(ElevatorConstants.elevatorkP, ElevatorConstants.elevatorkI, ElevatorConstants.elevatorkD);

    public Elevator(){
        kBrakeGeneralConfig
            .idleMode(IdleMode.kBrake);
        kBrakeGeneralConfig.encoder
            .positionConversionFactor(1.7262435)
            .velocityConversionFactor(1.7262435);
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

        elevatorMotorsToBrake();
    }

    public enum ElevatorPosition{
        L1,
        L2,
        L3,
        L4,
        PICKUP
    }

    public void elevatorMotorsToBrake(){
        // Left motor MUST be inverted
        elevatorLeftMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Right motor MUST NOT be inverted
        elevatorRightMotor.configure(kBrakeGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void elevatorMotorsToCoast(){
        elevatorLeftMotor.configure(kCoastGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorRightMotor.configure(kCoastGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stopElevatorMotors(){
        elevatorLeftMotor.set(0);
        elevatorRightMotor.set(0);
    }

    public double getElevatorPosition(){
        double elevatorPosition = (elevatorRightMotor.getEncoder().getPosition() / 
                                    elevatorRightMotor.getEncoder().getPosition()) * 2;
        return elevatorPosition;
    }

    private double getElevatorVelocity(){
        double elevatorVelocity = (elevatorRightMotor.getEncoder().getVelocity() / 
                                    elevatorRightMotor.getEncoder().getVelocity()) * 2;
        return elevatorVelocity;
    }

    private boolean getUpperLimitSwitch(){
        return !upperLimitSwitch.get();
    }

    private boolean getLowerLimitSwitch(){
        return !lowerLimitSwitch.get();
    }

    private double desaturatePidValue(double PID_value){
        if(PID_value > ElevatorConstants.elevatorMotorsMaxOutput){
            PID_value =  ElevatorConstants.elevatorMotorsMaxOutput;
        }
        else if(PID_value < -ElevatorConstants.elevatorMotorsMaxOutput){
            PID_value = -ElevatorConstants.elevatorMotorsMaxOutput;
        }
        return PID_value;
    }

    public void moveElevator(double velocity, boolean limitSecuritySystem){
        elevatorLeftMotor.set(velocity);
        elevatorRightMotor.set(velocity);

        if (getLowerLimitSwitch() && limitSecuritySystem && velocity < 0){
            stopElevatorMotors();
        }
        else if (getUpperLimitSwitch() && limitSecuritySystem && velocity > 0){
            stopElevatorMotors();
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Lower limit switch", getUpperLimitSwitch());
        SmartDashboard.putBoolean("Upper limit switch", getLowerLimitSwitch());
        SmartDashboard.putNumber("Elevator position", elevatorLeftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator velocity", elevatorLeftMotor.getEncoder().getVelocity());
    }
}
