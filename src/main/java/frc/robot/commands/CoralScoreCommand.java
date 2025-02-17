package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    }

    @Override
    public void end(boolean interrupted) {
        coralIntake.stopCoralIntakeMotors();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }