package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberMode;

public class Climbr extends SubsystemBase {
    
    private SparkMax leftClimberMotor = new SparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
    private SparkMax rightClimberMotor = new SparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);

    private SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private final PIDController climberPID = new PIDController(0.3, 0, 0);
    private double goal = 0;
    private double PIDvalue;
    private String goalClimberPosition = "Initial Position";

    public Climbr() {
        climberPID.setTolerance(0);
        
        kBrakeGeneralConfig
            .idleMode(IdleMode.kBrake);
        kBrakeGeneralConfig.encoder
            .positionConversionFactor(0.1)
            .velocityConversionFactor(0.1);

        kCoastGeneralConfig
            .idleMode(IdleMode.kCoast);
        kBrakeGeneralConfig.encoder
            .positionConversionFactor(0.1)
            .velocityConversionFactor(0.1);

        motorsToBrake();
    }

    public void motorsToBrake() {
        // Left motor MUST be inverted
        leftClimberMotor.configure(kBrakeGeneralConfig.inverted(false), null, PersistMode.kNoPersistParameters);
        // Right motor MUST NOT be inverted
        rightClimberMotor.configure(kBrakeGeneralConfig.inverted(false), null, PersistMode.kNoPersistParameters);
    }

    public void motorsToCoast() {
        leftClimberMotor.configure(kCoastGeneralConfig.inverted(false), null, PersistMode.kNoPersistParameters);
        rightClimberMotor.configure(kCoastGeneralConfig.inverted(false), null, PersistMode.kNoPersistParameters);
    }

    public void stopClimberMotors() {
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
    
    private void moveClimberMotors(double velocity) {
        leftClimberMotor.set(velocity);
        rightClimberMotor.set(velocity);
    }

    private double desaturatePidValue(double PID_value) {
        if(PID_value > ClimberConstants.climberMaxPositiveOutput){
            PID_value =  ClimberConstants.climberMaxPositiveOutput;
        } else if(PID_value < ClimberConstants.climberMaxNegativeOutput){
            PID_value = ClimberConstants.climberMaxNegativeOutput;
        }
        return PID_value;
    }

    private double getClimberPosition() {
        double climberPosition = (leftClimberMotor.getEncoder().getPosition() + rightClimberMotor.getEncoder().getPosition()) / 2;
        return climberPosition;
    }

    private boolean isAtTarget() {
        return climberPID.atSetpoint();
    }

    public void moveClimber(ClimberMode climberMode) {
        switch (climberMode) {
            case HOME:
                goal = ClimberConstants.HOMEPosition;
                goalClimberPosition = "Climber home position " + ClimberConstants.HOMEPosition;    
                break;
            case CLIMB:
                goal = ClimberConstants.CLIMBPosition;
                goalClimberPosition = "Climber climb position " + ClimberConstants.CLIMBPosition;    
                break;
        }
    }

    @Override
    public void periodic() {
        // goal = (goal > 0) ? goal : 0;
        goal = (goal < ClimberConstants.climberMaxExtension) ? goal : ClimberConstants.climberMaxExtension;

        PIDvalue = climberPID.calculate(getClimberPosition(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveClimberMotors(PIDvalue);

        SmartDashboard.putNumber("Climber position", getClimberPosition());
        SmartDashboard.putNumber("Climber PID value", PIDvalue);
        SmartDashboard.putBoolean("Climber is at target", isAtTarget());
        SmartDashboard.putString("Climber goal", goalClimberPosition);
    }
}
