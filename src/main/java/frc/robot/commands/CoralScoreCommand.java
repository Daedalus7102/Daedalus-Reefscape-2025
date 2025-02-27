package frc.robot.Commands;

/* 
import com.pathplanner.lib.path.IdealStartingState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intakes.CoralIntakeConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intakes.CoralIntake;

public class CoralScoreCommand extends Command {
    private final CoralIntake coralIntake;
    private CoralIntakeMode coralIntakeMode;
    private boolean isFinished;
    private Elevator elevator;
    Timer timer = new Timer();

    public CoralScoreCommand(CoralIntake coralIntake, CoralIntakeMode coralIntakeMode){
        this.coralIntake = coralIntake;
        this.coralIntakeMode = coralIntakeMode;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize(){
        coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeSecureCoralVelocity, true);
    }

    @Override
    public void execute(){
        // coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeSecureCoralVelocity, false);
        timer.start();

        switch (coralIntakeMode) {
            case HOME:
                coralIntake.moveCoralIntake(coralIntakeMode);
                if (!coralIntake.getInfraredSensorValue()){
                    coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeSecureCoralVelocity, false);
                } else {
                    coralIntake.stopIntakeMotors();
                }
                isFinished = false;
                break;

            case INTAKE_PICKUP:
                coralIntake.moveCoralIntake(coralIntakeMode);
                coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeIntakeVleocity, false);
                isFinished = coralIntake.getInfraredSensorValue();
                break;

            case L1_EJECT:
                coralIntake.moveCoralIntake(coralIntakeMode);
                if (coralIntake.pivotMotorInDesiredAngle() && elevator.isAtTarget()) {
                    timer.start();
                    if (timer.get() >= 0.8){
                    coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity, false);
                    }
                }
                isFinished = coralIntake.getInfraredSensorValue();
                break;

            case L2_AND_L3EJECT:
                coralIntake.moveCoralIntake(coralIntakeMode);
                if (coralIntake.pivotMotorInDesiredAngle() && elevator.isAtTarget()) {
                    timer.start();
                    if (timer.get() >= 0.8 && elevator.isAtTarget()){
                      coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity, false);
                    }
                }
                isFinished = coralIntake.getInfraredSensorValue();
                break;

            case L4_EJECT:
                coralIntake.moveCoralIntake(coralIntakeMode);
                if (coralIntake.pivotMotorInDesiredAngle() && elevator.isAtTarget()) {
                    timer.start();
                    if (timer.get() >= 0.8){
                        coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity, false);
                    }
                }
                isFinished = coralIntake.getInfraredSensorValue();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralIntake.stopPivotMotor();
        coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeSecureCoralVelocity, false);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
      return timer.get() > 3; //isFinished;
    }
  }
*/