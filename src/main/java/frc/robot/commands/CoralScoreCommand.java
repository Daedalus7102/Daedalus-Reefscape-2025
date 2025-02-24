package frc.robot.commands;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intakes.CoralIntakeConstants;
import frc.robot.subsystems.Intakes.CoralIntake;
import frc.robot.subsystems.Intakes.CoralIntake.CoralIntakeMode;

public class CoralScoreCommand extends Command {
    private final CoralIntake coralIntake;
    private final CoralIntakeMode coralIntakeMode;

    public CoralScoreCommand(CoralIntake coralIntake, CoralIntakeMode coralIntakeMode){
        this.coralIntake = coralIntake;
        this.coralIntakeMode = coralIntakeMode;
        // addRequirements(coralIntake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        coralIntake.moveCoralIntake(coralIntakeMode);
        if (coralIntake.getCoralIntakePivotAngle() > 78 && coralIntake.getCoralIntakePivotAngle() < 85) {
            coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralIntake.stopPivotMotor();
        coralIntake.stopIntakeMotors();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }