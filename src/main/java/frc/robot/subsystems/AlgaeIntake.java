package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants;
import frc.robot.Utilities.PositionEnums.AlgaeScorePositions;

import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntake extends SubsystemBase{
    private final SparkMax algaeIntakePivotMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakePivotMotorID, MotorType.kBrushless);

    private final SparkMax algaeIntakeLeftMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakeLeftMotorID, MotorType.kBrushless);
    private final SparkMax algaeIntakeRightMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakeRightMotorID, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private final CANcoder algaePivotCancoder = new CANcoder(AlgaeIntakeConstants.algaePivotCancoderID, "Drivetrain");
    private final PIDController algaePivotPID = new PIDController(AlgaeIntakeConstants.algaePivotkP, AlgaeIntakeConstants.algaePivotkI, AlgaeIntakeConstants.algaePivotkD);

    private final DigitalInput infraredSensor = new DigitalInput(2);

    private double goal = AlgaeIntakeConstants.HOMEPosition;
    private double PIDvalue;
    private String goalAlgaeIntakePosition = "Initial position";

    public AlgaeIntake(){
        kBrakeGeneralConfig
            .idleMode(IdleMode.kBrake);
        kCoastGeneralConfig
            .idleMode(IdleMode.kCoast);

        pivotMotorToCoast();
        // Algae left intake motor MUST be inverted
        algaeIntakeLeftMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Algae left intake motor MUST NOT be inverted
        algaeIntakeRightMotor.configure(kBrakeGeneralConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotMotorToBrake(){
        // Pivot motor on coral intake must be inverted
        algaeIntakePivotMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotMotorToCoast(){
        // Pivot motor on coral intake must be inverted
        algaeIntakePivotMotor.configure(kCoastGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean getInfraredSensorValue() {
        return !infraredSensor.get();
    }

    public void stopPivotMotor(){
        algaeIntakePivotMotor.set(0);
    }

    public void stopIntakeMotors(){
        algaeIntakeLeftMotor.set(0);
        algaeIntakeRightMotor.set(0);
    }

    public void moveAlgaePivotMotor(double velocity){
        algaeIntakePivotMotor.set(velocity);
    }

    public void moveAlgaeIntakeMotors(double velocity, boolean securitySystem){
        if(securitySystem) {
            if (getInfraredSensorValue()) {
                stopIntakeMotors();
            } else {
                algaeIntakeLeftMotor.set(velocity);
                algaeIntakeRightMotor.set(velocity);
            }
        } else {
            algaeIntakeLeftMotor.set(velocity);
            algaeIntakeRightMotor.set(velocity);
        }
    }

    public double getAlgaeIntakePivotAngle() {
        double algaePivotAngle = algaePivotCancoder.getAbsolutePosition().getValueAsDouble() * 360 + AlgaeIntakeConstants.algaePivotEncoderOffset;
        return algaePivotAngle;
    }

    private double desaturatePidValue(double PID_value) {
        if(PID_value > AlgaeIntakeConstants.algaePivotMotorMaxPositiveOutPut){
            PID_value =  AlgaeIntakeConstants.algaePivotMotorMaxPositiveOutPut;
        }
        else if(PID_value < AlgaeIntakeConstants.algaePivotMotorMaxNegativeOutput){
            PID_value = AlgaeIntakeConstants.algaePivotMotorMaxNegativeOutput;
        }
        return PID_value;
    }

    public enum AlgaeIntakeMode{
        FLOOR_INTAKE,
        PROCCESOR_EJECT,
        BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position,
        HOME,
        NET_EJECT,
    }

    public void moveAlgaeIntake(AlgaeScorePositions scorePositions){
        goal = scorePositions.getAlgaeIntakePosition();

        goal = (goal > AlgaeIntakeConstants.pivotMinAngle) ? goal : AlgaeIntakeConstants.pivotMinAngle;
        goal = (goal < AlgaeIntakeConstants.pivotMaxAngle) ? goal : AlgaeIntakeConstants.pivotMaxAngle;
    }

    public boolean pivotMotorInDesiredAngle() {
        boolean pivotMotorAngleApplied = getAlgaeIntakePivotAngle() <= goal + AlgaeIntakeConstants.pivotMotorkDeadBand
                                        || getAlgaeIntakePivotAngle() >= goal - AlgaeIntakeConstants.pivotMotorkDeadBand;
        return pivotMotorAngleApplied;
    }

    @Override
    public void periodic(){
        PIDvalue = algaePivotPID.calculate(getAlgaeIntakePivotAngle(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveAlgaePivotMotor(PIDvalue);

        SmartDashboard.putNumber("Algae intake pivot angle", getAlgaeIntakePivotAngle());
        SmartDashboard.putBoolean("Algae intake infrared sensor", getInfraredSensorValue());
        SmartDashboard.putNumber("Algae intake pivot PID", PIDvalue);
        SmartDashboard.putString("Algae intake pivot Goal", goalAlgaeIntakePosition);
        SmartDashboard.putBoolean("Algae pivot motor to desired angle", pivotMotorInDesiredAngle());
        SmartDashboard.putNumber("Coral pivot motor speed", algaeIntakePivotMotor.getEncoder().getVelocity());
    }
}
