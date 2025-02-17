package frc.robot.subsystems.Intakes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntake extends SubsystemBase{
    private final SparkMax algaeIntakeLeftMotor = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax algaeIntakeRightMotor = new SparkMax(12, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeAlgaeIntakeConfig = new SparkMaxConfig();

    public AlgaeIntake(){
        kBrakeAlgaeIntakeConfig.idleMode(IdleMode.kBrake);

        algaeIntakeLeftMotor.configure(kBrakeAlgaeIntakeConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntakeRightMotor.configure(kBrakeAlgaeIntakeConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
