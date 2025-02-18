package frc.robot.subsystems.Intakes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intakes.CoralIntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntake extends SubsystemBase{
    private final SparkMax coralIntakePivotMotor = new SparkMax(14, MotorType.kBrushless);

    private final SparkMax coralIntakeLeftMotor = new SparkMax(15, MotorType.kBrushless);
    private final SparkMax coralIntakeRightMotor = new SparkMax(16, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeCoralIntakeConfig = new SparkMaxConfig();
    
    public CoralIntake(){
        kBrakeCoralIntakeConfig.idleMode(IdleMode.kBrake);

        coralIntakeLeftMotor.configure(kBrakeCoralIntakeConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralIntakeRightMotor.configure(kBrakeCoralIntakeConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum CoralIntakeMode{
        INTAKE,
        EJECT
    }

    public void stopCoralIntakeMotors(){
        coralIntakeLeftMotor.set(0);
        coralIntakeRightMotor.set(0);
    }

    public void moveCoralIntakeMotors(double velocity){
        coralIntakeLeftMotor.set(velocity);
        coralIntakeRightMotor.set(velocity);
    }

    public void moveCoralIntake(CoralIntakeMode coralintakeMode){
        switch (coralintakeMode) {
            case INTAKE:
                moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeIntakeVleocity);
                break;

            case EJECT:
                moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity);
                break;
        }
    }

    @Override
    public void periodic(){}
}
