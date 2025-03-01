package frc.robot.subsystems.Intakes;

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

import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntake extends SubsystemBase{
    private final SparkMax algaeIntakePivotMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakePivotMotorID, MotorType.kBrushless);

    private final SparkMax algaeIntakeLeftMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakeLeftMotorID, MotorType.kBrushless);
    private final SparkMax algaeIntakeRightMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakeRightMotorID, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private final CANcoder algaePivotCancoder = new CANcoder(AlgaeIntakeConstants.algaePivotCancoderID, "Drivetrain");
    private final PIDController algaePivotPID = new PIDController(AlgaeIntakeConstants.algaePivotkP, AlgaeIntakeConstants.algaePivotkI, AlgaeIntakeConstants.algaePivotkD);

    private final DigitalInput infraredSensor = new DigitalInput(1);

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

    public enum AlgaeIntakeMode{
        FLOOR_INTAKE,
        PROCCESOR_EJECT,
        BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position,
        HOME,
        HOME_WITH_ALGAE,
        NET_EJECT,
    }

    public void pivotMotorToBrake(){
        // Pivot motor on coral intake must be inverted
        algaeIntakePivotMotor.configure(kBrakeGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotMotorToCoast(){
        // Pivot motor on coral intake must be inverted
        algaeIntakePivotMotor.configure(kCoastGeneralConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public boolean getInfraredSensorValue() {
        return !infraredSensor.get();
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

    public void stopPivotMotor(){
        algaeIntakePivotMotor.set(0);
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

    public void moveAlgaeIntake(AlgaeIntakeMode algaeIntakeMode){
        switch (algaeIntakeMode) {
            case FLOOR_INTAKE:
                goal = AlgaeIntakeConstants.FLOOR_INTAKEPosition;
                goalAlgaeIntakePosition = "Algae intake floor " + AlgaeIntakeConstants.FLOOR_INTAKEPosition;
                break;
            case PROCCESOR_EJECT:
                goal = AlgaeIntakeConstants.PROCCESOR_EJECTPosition;
                goalAlgaeIntakePosition = "Algae intake processor " + AlgaeIntakeConstants.PROCCESOR_EJECTPosition;
                break;
            case BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position:
                goal = AlgaeIntakeConstants.BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position;
                goalAlgaeIntakePosition = "Algae intake Between L2 and L3 or L3 and L4 " + AlgaeIntakeConstants.BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position;
                break;
            case HOME:
                goal = AlgaeIntakeConstants.HOMEPosition;
                goalAlgaeIntakePosition = "Algae intake Home position " + AlgaeIntakeConstants.HOMEPosition;
                break;
            case HOME_WITH_ALGAE:
                goal = AlgaeIntakeConstants.HOME_WITH_ALGAEPosition;
                goalAlgaeIntakePosition = "Algae intake Home with algae position " + AlgaeIntakeConstants.HOME_WITH_ALGAEPosition;
                break;
            case NET_EJECT:
                goal = AlgaeIntakeConstants.NET_EJECTPosition;
                goalAlgaeIntakePosition = "Algae intake Net eject position " + AlgaeIntakeConstants.NET_EJECTPosition;
                break;
        }
    }

    public boolean pivotMotorInDesiredAngle() {
        boolean pivotMotorAngleApplied = getAlgaeIntakePivotAngle() <= goal + AlgaeIntakeConstants.pivotMotorkDeadBand
                                        || getAlgaeIntakePivotAngle() >= goal - AlgaeIntakeConstants.pivotMotorkDeadBand;
        return pivotMotorAngleApplied;
    }

    @Override
    public void periodic(){
        goal = (goal > AlgaeIntakeConstants.pivotMinAngle) ? goal : AlgaeIntakeConstants.pivotMinAngle;
        goal = (goal < AlgaeIntakeConstants.pivotMaxAngle) ? goal : AlgaeIntakeConstants.pivotMaxAngle;

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
