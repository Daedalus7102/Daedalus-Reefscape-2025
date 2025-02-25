package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.AimbotConstants;
import frc.robot.Constants.SwerveConstants.AutoRotateConstants;
import frc.robot.subsystems.Drive.Swerve;

public class AimbotCommand extends Command{
  private final Swerve swerve;
  private boolean needsRotCalibration;
  private double angleError;

  private double xAprilTagTarget;
  private double yAprilTagTarget;
  private boolean algined = false;
  Timer timer = new Timer();

  public AimbotCommand(Swerve swerve) {
      this.swerve = swerve;
      addRequirements(swerve);
    }
  
  @Override
  public void initialize() {}

  private enum LeftRightTarget {
    LEFT,
    RIGHT
  }

  // No need of limelight for these functions
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
      needsRotCalibration = Math.abs(swerve.getNormalizedGyroAngle() - setPoint) > SwerveConstants.AutoRotateConstants.autoRotationkDeadband;
      
      if(needsRotCalibration){
          angleError = setPoint - swerve.getNormalizedGyroAngle();
          if(setPoint == 0 && swerve.getNormalizedGyroAngle() > 330){
              angleError = 360 - swerve.getNormalizedGyroAngle();
          }
          double speed = angleError * SwerveConstants.AutoRotateConstants.chassisZAutoRotationkP;
          return speed;
      }
      return 0.0;
  }

  // Limelight needed for the following functions
  private void selectLeftRightTarget(LeftRightTarget leftRightTarget) {
    switch (leftRightTarget) {
      case LEFT:
        xAprilTagTarget = AimbotConstants.leftTargetTXAimbotReef;
        yAprilTagTarget = AimbotConstants.leftTargetTYAimbotReef;
        break;
    
      case RIGHT:
        xAprilTagTarget = AimbotConstants.rightTargetTXAimbotReef;
        yAprilTagTarget = AimbotConstants.rightTargetTYAimbotReef;
        break;
    }
  }

  private boolean needsXCalibration(double tagPosTx) {
    if(tagPosTx > 0) {
      selectLeftRightTarget(LeftRightTarget.LEFT);
    } else {
      selectLeftRightTarget(LeftRightTarget.RIGHT);
    }
    
    boolean needsCalibration = tagPosTx <= xAprilTagTarget - AimbotConstants.xAprilTagThreshold 
                            || tagPosTx >= xAprilTagTarget + AimbotConstants.xAprilTagThreshold;
    return needsCalibration;
  }

  private boolean needsYCalibration(double tagPosTy) {
    boolean needsCalibration = tagPosTy <= yAprilTagTarget - AimbotConstants.yApriltagThreshold 
                            || tagPosTy >= yAprilTagTarget + AimbotConstants.yApriltagThreshold;
    return needsCalibration;
  }

  public double calibrateX(double tagPosTx) {
    if(needsXCalibration(tagPosTx)){
      double error = (xAprilTagTarget - tagPosTx) * Math.signum(tagPosTx); // Calculate error
      double speed = AimbotConstants.kPdriveX * AimbotConstants.xDriveMaxSpeed * error / xAprilTagTarget; // Adjust velocity

      if(speed > AimbotConstants.xDriveMaxSpeed){
        speed = AimbotConstants.xDriveMaxSpeed;
      }
      return speed;
    }
    return 0.0;
  }

  public double calibrateY(double tagPosTy) {
    if (needsYCalibration(tagPosTy)) {
      double error = (yAprilTagTarget - tagPosTy); // Calculate error
      double speed = AimbotConstants.kPdriveY * AimbotConstants.yDriveMaxSpeed * error / yAprilTagTarget; // Adjust velocity
      
      if(speed > AimbotConstants.yDriveMaxSpeed){
        speed = AimbotConstants.yDriveMaxSpeed;
      }
      return speed;
    }
    return 0.0;
  }

  @Override
  public void execute() {
    timer.start();
    double zSpeed = calibrateZ(desiredAngle());
    swerve.setChassisSpeeds(0, 0, zSpeed, AutoRotateConstants.autoRotationMaxOutput, true);
    SmartDashboard.putBoolean("Needs rot calibration", needsRotCalibration);
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    
    SmartDashboard.putNumber("Aimbot xTarget", xAprilTagTarget);
    SmartDashboard.putNumber("Aimbot yTarget", yAprilTagTarget);

    // Read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double found = table.getEntry("tv").getDouble(0);
    
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);

    if(found == 1 && timer.get() > 0.5){
        double xSpeed = calibrateX(x);
        double ySpeed = calibrateY(y);
        swerve.setChassisSpeeds(ySpeed, xSpeed, zSpeed, 0.2, true);

        SmartDashboard.putBoolean("Aimbot Needs x calibration", needsXCalibration(x));
        SmartDashboard.putBoolean("Aimbot Needs y calibration", needsYCalibration(y));  
        SmartDashboard.putNumber("Aimbot xSpeed", xSpeed);
        SmartDashboard.putNumber("Aimbot ySpeed", ySpeed);  
        SmartDashboard.putNumber("Aimbot Required distance X", x - AimbotConstants.leftTargetTXAimbotReef);
        SmartDashboard.putNumber("Aimbot Required distance Y", y - AimbotConstants.rightTargetTYAimbotReef);    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(0, 0, 0, 0, true);
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
