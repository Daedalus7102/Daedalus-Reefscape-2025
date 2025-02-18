package frc.robot.subsystems.Intakes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntake extends SubsystemBase{
    private final SparkMax algaeIntakePivotMotor = new SparkMax(11, MotorType.kBrushless);

    private final SparkMax algaeIntakeLeftMotor = new SparkMax(12, MotorType.kBrushless);
    private final SparkMax algaeIntakeRightMotor = new SparkMax(13, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeAlgaeIntakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastAlgaeIntakeConfig = new SparkMaxConfig();

    public AlgaeIntake(){
        kBrakeAlgaeIntakeConfig
            .idleMode(IdleMode.kBrake);
        kCoastAlgaeIntakeConfig
            .idleMode(IdleMode.kCoast);

        algaeIntakePivotMotorToBrake();

        algaeIntakeLeftMotor.configure(kBrakeAlgaeIntakeConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntakeRightMotor.configure(kBrakeAlgaeIntakeConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void algaeIntakePivotMotorToBrake(){
        // Pivot motor on coral intake must be inverted
        algaeIntakePivotMotor.configure(kBrakeAlgaeIntakeConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void elevatorMotorsToCoast(){
        // Pivot motor on coral intake must be inverted
        algaeIntakePivotMotor.configure(kCoastAlgaeIntakeConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum AlgaeIntakeMode{
        INTAKE,
        EJECT
    }

    public void stopAlgaeIntakeMotors(){
        algaeIntakeLeftMotor.set(0);
        algaeIntakeRightMotor.set(0);
    }

    public void moveAlgaeIntakeMotors(double velocity){
        algaeIntakeLeftMotor.set(velocity);
        algaeIntakeRightMotor.set(velocity);
    }

    public void pivotAlgaeIntake(double velocity){
        algaeIntakePivotMotor.set(velocity);
    }

    public void stopPivotAlgaeIntakeMotor(){
        algaeIntakePivotMotor.set(0);
    }

    public void moveAlgaeIntake(AlgaeIntakeMode algaeIntakeMode){
        switch (algaeIntakeMode) {
            case INTAKE:
                moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeIntakeVelocity);
                break;

            case EJECT:
                moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeEjectVelocity);
                break;
        }
    }

    @Override
    public void periodic(){}
}
