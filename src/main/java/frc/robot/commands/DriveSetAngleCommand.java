package frc.robot.commands;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Drive.Swerve;

public class DriveSetAngleCommand extends Command {
    Swerve swerve;
    double angleError;

    public DriveSetAngleCommand(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    private double angleError(double chassisAngle){
        angleError = chassisAngle / 6;

        if (angleError < 30){
            angleError = angleError * -1;
        }
        return angleError;
    }

    private void adjustZ(){
        boolean needsCalibration = (angleError - swerve.getAngle()) > SwerveDriveConstants.autoRotationkDeadband
                                ||(angleError + swerve.getAngle()) > SwerveDriveConstants.autoRotationkDeadband;
        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        angleError(swerve.getAngle());

    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }

}
