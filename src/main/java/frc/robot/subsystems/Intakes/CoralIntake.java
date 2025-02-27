package frc.robot.subsystems.Intakes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Intakes.CoralIntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntake extends SubsystemBase{
    private final SparkMax coralIntakePivotMotor = new SparkMax(CoralIntakeConstants.coralIntakePivotMotorID, MotorType.kBrushless);

    private final SparkMax coralIntakeLeftMotor = new SparkMax(CoralIntakeConstants.coralIntakeLeftMotorID, MotorType.kBrushless);
    private final SparkMax coralIntakeRightMotor = new SparkMax(CoralIntakeConstants.coralIntakeRightMotorID, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private final CANcoder coralPivotCancoder = new CANcoder(CoralIntakeConstants.coralPivotCancoderID, "Drivetrain");
    private final PIDController coralPivotPID = new PIDController(CoralIntakeConstants.coralPivotkP, CoralIntakeConstants.coralPivotkI, CoralIntakeConstants.coralPivotkD);

    private final DigitalInput infraredSensor = new DigitalInput(0);

    private double goal = CoralIntakeConstants.HOMEPosition;
    private double PIDvalue;
    private String goalCoralIntakePosition = "Initial position";
    
    public CoralIntake(){
        kBrakeGeneralConfig
            .idleMode(IdleMode.kBrake);
        kCoastGeneralConfig
            .idleMode(IdleMode.kCoast);
        
        pivotMotorToBrake();

        // Coral left intake motor MUST be inverted
        coralIntakeLeftMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Coral right intake motor MUST NOT be inverted
        coralIntakeRightMotor.configure(kBrakeGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum CoralIntakeMode{
        HOME,
        INTAKE_PICKUP,
        L1_EJECT,
        L2_AND_L3EJECT,
        L4_EJECT
    }

    public void pivotMotorToBrake(){
        // Pivot motor on coral intake MUST NOT be inverted
        coralIntakePivotMotor.configure(kBrakeGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotMotorToCoast(){
        // Pivot motor on coral intake MUST NOT be inverted
        coralIntakePivotMotor.configure(kCoastGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean getInfraredSensorValue() {
        return !infraredSensor.get();
    }

    public void stopIntakeMotors(){
        coralIntakeLeftMotor.set(0);
        coralIntakeRightMotor.set(0);
    }

    public void moveCoralPivotMotor(double velocity){
        coralIntakePivotMotor.set(velocity);
    }

    public void moveCoralIntakeMotors(double velocity, boolean securitySystem){
        if(securitySystem) {
            if (getInfraredSensorValue()) {
                stopIntakeMotors();
            } else {
                coralIntakeLeftMotor.set(velocity);
                coralIntakeRightMotor.set(velocity);    
            }
        } else {
            coralIntakeLeftMotor.set(velocity);
            coralIntakeRightMotor.set(velocity);
        }
    }

    public void stopPivotMotor(){
        coralIntakePivotMotor.set(0);
    }

    public double getCoralIntakePivotAngle(){
        double coralPivotAngle = coralPivotCancoder.getAbsolutePosition().getValueAsDouble() * 360 + CoralIntakeConstants.coralPivotEncoderOffset;
        return coralPivotAngle;
    }

    private double desaturatePidValue(double PID_value){
        if(getInfraredSensorValue()) {
            CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut =+ 0.2;
        } else {
            if(getCoralIntakePivotAngle() > 150) {
                CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut = 0.05;
            }
            else {
                CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut = 0.3;
            }
        }

        if(PID_value > CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut){
            PID_value =  CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut;
        }
        else if(PID_value < CoralIntakeConstants.coralPivotMotorMaxNegativeOutput){
            PID_value = CoralIntakeConstants.coralPivotMotorMaxNegativeOutput;
        }
        return PID_value;
    }

    public void moveCoralIntake(CoralIntakeMode coralintakeMode){
        switch (coralintakeMode) {
            case HOME:
                goal = CoralIntakeConstants.HOMEPosition;
                goalCoralIntakePosition = "Coral Intake Home Deg" + CoralIntakeConstants.HOMEPosition;    
                break;
            case INTAKE_PICKUP:
                goal = CoralIntakeConstants.INTAKE_PICKUPPosition;
                goalCoralIntakePosition = "Coral Intake pickUp Deg" + CoralIntakeConstants.INTAKE_PICKUPPosition;
                break;
            case L1_EJECT:
                goal = CoralIntakeConstants.L1Position;
                goalCoralIntakePosition = "Coral Intake pickUp Deg" + CoralIntakeConstants.L1Position;
                break;
            case L2_AND_L3EJECT:
                goal = CoralIntakeConstants.L2_and_L3Position;
                goalCoralIntakePosition = "Coral Intake pickUp Deg" + CoralIntakeConstants.L2_and_L3Position;
                break;
            case L4_EJECT:
                goal = CoralIntakeConstants.L4Position;
                goalCoralIntakePosition = "Coral Intake pickUp Deg" + CoralIntakeConstants.L4Position;
                break;
        }                
    }            

    public boolean pivotMotorInDesiredAngle() {
        boolean pivotMotorAngleApplied = getCoralIntakePivotAngle() <= goal + CoralIntakeConstants.pivotMotorkDeadBand
                                        || getCoralIntakePivotAngle() >= goal - CoralIntakeConstants.pivotMotorkDeadBand;
        return pivotMotorAngleApplied;
    }

    @Override
    public void periodic(){
        goal = (goal > CoralIntakeConstants.pivotMinAngle) ? goal : CoralIntakeConstants.pivotMinAngle;
        goal = (goal < CoralIntakeConstants.pivotMaxAngle) ? goal : CoralIntakeConstants.pivotMaxAngle;

        PIDvalue = coralPivotPID.calculate(getCoralIntakePivotAngle(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveCoralPivotMotor(PIDvalue);

        SmartDashboard.putNumber("Coral intake pivot angle", getCoralIntakePivotAngle());
        SmartDashboard.putBoolean("Coral infrared sensor", getInfraredSensorValue());
        SmartDashboard.putNumber("Coral intake pivot PID", PIDvalue);
        SmartDashboard.putString("Coral Intake desired position", goalCoralIntakePosition);
        SmartDashboard.putBoolean("Coral pivot motor to desired angle", pivotMotorInDesiredAngle());
        SmartDashboard.putNumber("Coral pivot motor speed", coralIntakePivotMotor.getEncoder().getVelocity());
    }
}
