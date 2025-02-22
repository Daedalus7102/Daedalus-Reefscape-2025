package frc.robot.subsystems.Intakes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intakes.CoralIntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntake extends SubsystemBase{
    private final SparkMax coralIntakePivotMotor = new SparkMax(CoralIntakeConstants.coralIntakePivotMotorID, MotorType.kBrushless);

    private final SparkMax coralIntakeLeftMotor = new SparkMax(CoralIntakeConstants.coralIntakeLeftMotorID, MotorType.kBrushless);
    private final SparkMax coralIntakeRightMotor = new SparkMax(CoralIntakeConstants.coralIntakeRightMotorID, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private final CANcoder coralPivotCancoder = new CANcoder(CoralIntakeConstants.coralPivotCancoderID);
    private final PIDController coralPivotPID = new PIDController(CoralIntakeConstants.coralPivotkP, CoralIntakeConstants.coralPivotkI, CoralIntakeConstants.coralPivotkD);

    private double goal;
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

    public void pivotMotorToBrake(){
        // Pivot motor on coral intake must be inverted
        coralIntakePivotMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotMotorToCoast(){
        // Pivot motor on coral intake must be inverted
        coralIntakePivotMotor.configure(kCoastGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveCoralPivotMotor(double velocity){
        coralIntakePivotMotor.set(velocity);
    }

    public void moveCoralIntakeMotors(double velocity){
        coralIntakeLeftMotor.set(velocity);
        coralIntakeRightMotor.set(velocity);
    }

    public void stopPivotMotor(){
        coralIntakePivotMotor.set(0);
    }

    public void stopIntakeMotors(){
        coralIntakeLeftMotor.set(0);
        coralIntakeRightMotor.set(0);
    }

    public double getCoralIntakePivotAngle(){
        double coralPivotAngle = coralPivotCancoder.getAbsolutePosition().getValueAsDouble() * 360 + CoralIntakeConstants.coralPivotEncoderOffset;
        return coralPivotAngle;
    }

    private double desaturatePidValue(double PID_value){
        if(PID_value > CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut){
            PID_value =  CoralIntakeConstants.coralPivotMotorMaxPositiveOutPut;
        }
        else if(PID_value < CoralIntakeConstants.coralPivotMotorMaxNegativeOutput){
            PID_value = CoralIntakeConstants.coralPivotMotorMaxNegativeOutput;
        }
        return PID_value;
    }

    public enum CoralIntakeMode{
        INTAKE_PICKUP,
        L1_EJECT,
        L2_EJECT,
        L3_EJECT,
        L4_EJECT,
        EJECT
    }

    public void moveCoralIntake(CoralIntakeMode coralintakeMode){
        switch (coralintakeMode) {
            case INTAKE_PICKUP:
                goal = CoralIntakeConstants.PickUpGoalPosition;
                goalCoralIntakePosition = "Intake pickUp 0 Deg";
                break;
            case L1_EJECT:
                goal = CoralIntakeConstants.L1GoalPosition;
                goalCoralIntakePosition = "Intake pickUp 0 Deg";
                break;
            case L2_EJECT:
                goal = CoralIntakeConstants.L2_and_L3CoralGoalPosition;
                goalCoralIntakePosition = "Intake pickUp 0 Deg";
                break;
            case L3_EJECT:
                goal = CoralIntakeConstants.L2_and_L3CoralGoalPosition;
                goalCoralIntakePosition = "Intake pickUp 0 Deg";
                break;
            case L4_EJECT:
                goal = CoralIntakeConstants.L4GoalPosition;
                goalCoralIntakePosition = "Intake pickUp 0 Deg";
                break;
            case EJECT:
                moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity);
                break;
        }

        PIDvalue = coralPivotPID.calculate(getCoralIntakePivotAngle(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveCoralPivotMotor(PIDvalue);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Coral intake pivot angle", getCoralIntakePivotAngle());
        SmartDashboard.putNumber("Coral intake pivot PID", PIDvalue);
        SmartDashboard.putString("Coral Intake desired position", goalCoralIntakePosition);
    }
}
