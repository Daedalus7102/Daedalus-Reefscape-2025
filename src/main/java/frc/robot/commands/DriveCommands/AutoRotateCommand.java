package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoRotateConstants;
import frc.robot.subsystems.Drive.Swerve;

public class AutoRotateCommand extends Command {
    private final Swerve swerve;
    private double angleError;
    private boolean needsCalibration;

    public AutoRotateCommand(Swerve swerve, boolean rotate180Deg) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    private int desiredAngle(){
        int setPoints[] = {0, 60, 120, 180, 240, 300};
        
        // Returns from 0 - 5 the array position to get the desired angle
        int angleArrayPosition = (int) Math.round(swerve.getNormalizedGyroAngle() / 60);
        if(angleArrayPosition == 6){
            angleArrayPosition = 0;
        }
        return setPoints[angleArrayPosition];
    }

    private double calibrateZ(int setPoint){
        needsCalibration = Math.abs(swerve.getNormalizedGyroAngle() - setPoint) > AutoRotateConstants.autoRotationkDeadband;
        
        if(needsCalibration){
            angleError = setPoint - swerve.getNormalizedGyroAngle();
            if(setPoint == 0 && swerve.getNormalizedGyroAngle() > 300){
                angleError = 360 - swerve.getNormalizedGyroAngle();
            }
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
        SmartDashboard.putNumber("array position without rounding", swerve.getNormalizedGyroAngle());
        SmartDashboard.putNumber("angle error", angleError);
        double zSpeed = calibrateZ(desiredAngle());
        swerve.setChassisSpeeds(0, 0, zSpeed, true, AutoRotateConstants.autoRotationMaxOutput);
        SmartDashboard.putNumber("speed", zSpeed);
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return !needsCalibration;
    }

}
