package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    private final SparkMax elevatorLeftMotor = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax elevatorRightMotor = new SparkMax(10, MotorType.kBrushless);

    private final DigitalInput upperLimitSwitch = new DigitalInput(1);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(0);

    private final SparkMaxConfig kBrakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastConfig = new SparkMaxConfig();

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.elevatorkP, ElevatorConstants.elevatorkI, ElevatorConstants.elevatorkD);

    public Elevator(){
        kBrakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        kBrakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        kCoastConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        kCoastConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }

    private enum elevatorPosition{
        L1,
        L2,
        L3,
        L4,
        PICKUP
    }

    public void elevatorMotorsToCoast(){
        elevatorLeftMotor.configure(kCoastConfig, null, PersistMode.kPersistParameters);
        elevatorRightMotor.configure(kCoastConfig, null, PersistMode.kPersistParameters);
    }

    public void elevatorMotorsToBrake(){
        elevatorLeftMotor.configure(kBrakeConfig, null, PersistMode.kPersistParameters);
        elevatorRightMotor.configure(kBrakeConfig, null, PersistMode.kPersistParameters);
    }

    public void stopElevatorMotors(){
        elevatorLeftMotor.set(0);
        elevatorRightMotor.set(0);
    }

    public double getElevatorPosition(){
        double elevatorPosition = elevatorRightMotor.getEncoder().getPosition();
        return elevatorPosition;
    }

    private boolean getUpperLimitSwitch(){
        return upperLimitSwitch.get();
    }

    private boolean getLowerLimitSwitch(){
        return lowerLimitSwitch.get();
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

    public void moveElevatorMotors(){
        elevatorLeftMotor.set(0.1);
        elevatorRightMotor.set(0.1);
    }

    public void periodic(){
        SmartDashboard.putNumber("Encoder value", getElevatorPosition());
    }
}
