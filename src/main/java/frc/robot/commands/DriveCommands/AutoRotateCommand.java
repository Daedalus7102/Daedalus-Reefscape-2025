package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoRotateConstants;
import frc.robot.subsystems.Drive.Swerve;

public class AutoRotateCommand extends Command {
    private final Swerve swerve;
    private double angleError;
    private boolean needsCalibration;
    private boolean rotate180Deg;

    public AutoRotateCommand(Swerve swerve, boolean rotate180Deg) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    private double getGyroValue(){
        double angle = swerve.gyro.getAngle() % 360;
        if (angle < 0){
            angle = angle + 360;
        }
        return angle;
    }

    private int desiredAngle(){
        int setPoints[] = {0, 60, 120, 180, 240, 320};
        
        // Returns from 0 - 5 the array position to get the desired angle
        int angleArrayPosition = (int) Math.round(getGyroValue() / 60);
        if(angleArrayPosition == 6){
            angleArrayPosition = 0;
        }
        return setPoints[angleArrayPosition];
    }

    private double calibrateZ(int setPoint){
        needsCalibration = Math.abs(getGyroValue() - setPoint) > AutoRotateConstants.autoRotationkDeadband;
        
        if(needsCalibration){
            angleError = setPoint - getGyroValue();
            double speed = angleError * AutoRotateConstants.chassisZAutoRotationkP;
            return speed;
        }
        return 0.0;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        SmartDashboard.putNumber("array position", desiredAngle());
        SmartDashboard.putNumber("array position without rounding", getGyroValue());
        SmartDashboard.putNumber("angle error", angleError);
        double zSpeed = calibrateZ(desiredAngle());
        swerve.setChassisSpeeds(0, 0, zSpeed, true, AutoRotateConstants.autoRotationMaxOutput);

    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }

}
