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
import frc.robot.Constants.Intakes.AlgaeIntakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeIntake extends SubsystemBase{
    private final SparkMax algaeIntakePivotMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakePivotMotorID, MotorType.kBrushless);

    private final SparkMax algaeIntakeLeftMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakeLeftMotorID, MotorType.kBrushless);
    private final SparkMax algaeIntakeRightMotor = new SparkMax(AlgaeIntakeConstants.algaeIntakeRightMotorID, MotorType.kBrushless);

    private final SparkMaxConfig kBrakeGeneralConfig = new SparkMaxConfig();
    private final SparkMaxConfig kCoastGeneralConfig = new SparkMaxConfig();

    private final CANcoder algaePivotCancoder = new CANcoder(AlgaeIntakeConstants.algaePivotCancoderID);
    private final PIDController algaePivotPID = new PIDController(AlgaeIntakeConstants.algaePivotkP, AlgaeIntakeConstants.algaePivotkI, AlgaeIntakeConstants.algaePivotkD);

    private double goal;
    private double PIDvalue;
    private String goalAlgaeIntakePosition = "Initial position;";

    public AlgaeIntake(){
        kBrakeGeneralConfig
            .idleMode(IdleMode.kBrake);
        kCoastGeneralConfig
            .idleMode(IdleMode.kCoast);

        pivotMotorToBrake();

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

    public void moveAlgaePivotMotor(double velocity){
        algaeIntakePivotMotor.set(velocity);
    }

    public void moveAlgaeIntakeMotors(double velocity){
        algaeIntakeLeftMotor.set(velocity);
        algaeIntakeRightMotor.set(velocity);
    }

    public void stopPivotMotor(){
        algaeIntakePivotMotor.set(0);
    }

    public void stopIntakeMotors(){
        algaeIntakeLeftMotor.set(0);
        algaeIntakeRightMotor.set(0);
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
        L2_INTAKE,
        L3_INTAKE,
        NET_EJECT,
        EJECT
    }

    public void moveAlgaeIntake(AlgaeIntakeMode algaeIntakeMode){
        switch (algaeIntakeMode) {
            case FLOOR_INTAKE:
                goal = AlgaeIntakeConstants.FloorIntakeGoalPosition;
                goalAlgaeIntakePosition = "Floor intake 0 Deg";
                break;
            case PROCCESOR_EJECT:
                goal = AlgaeIntakeConstants.ProccesorGoalPosition;
                goalAlgaeIntakePosition = "Floor intake 0 Deg";
                break;
            case L2_INTAKE:
                goal = AlgaeIntakeConstants.L2_and_L3AlgaeGoalPosition;
                goalAlgaeIntakePosition = "Floor intake 0 Deg";
                break;
            case L3_INTAKE:
                goal = AlgaeIntakeConstants.L2_and_L3AlgaeGoalPosition;
                goalAlgaeIntakePosition = "Floor intake 0 Deg";
                break;
            case NET_EJECT:
                goal = AlgaeIntakeConstants.NetEjectGoalPosition;
                goalAlgaeIntakePosition = "Floor intake 0 Deg";
                break;
            case EJECT:
                moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeEjectVelocity);
                break;
        }

        PIDvalue = algaePivotPID.calculate(getAlgaeIntakePivotAngle(), goal);
        PIDvalue = desaturatePidValue(PIDvalue);
        moveAlgaePivotMotor(PIDvalue);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae intake pivot angle", getAlgaeIntakePivotAngle());
        SmartDashboard.putNumber("Algae intake pivot PID", PIDvalue);
        SmartDashboard.putString("Algae intake pivot Goal", goalAlgaeIntakePosition);
    }
}
