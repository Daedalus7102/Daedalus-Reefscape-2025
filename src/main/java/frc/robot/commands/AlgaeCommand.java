package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intakes.AlgaeIntake;
import frc.robot.subsystems.Intakes.AlgaeIntake.AlgaeIntakeMode;

public class AlgaeCommand extends Command {
    private final AlgaeIntake algaeIntake;
    private final AlgaeIntakeMode algaeIntakeMode;

    public AlgaeCommand(AlgaeIntake algaeIntake, AlgaeIntakeMode algaeIntakeMode){
        this.algaeIntake = algaeIntake;
        this.algaeIntakeMode = algaeIntakeMode;
        // addRequirements(coralIntake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        algaeIntake.moveAlgaeIntake(algaeIntakeMode);
    }

    @Override
    public void end(boolean interrupted) {
        algaeIntake.stopAlgaeIntakeMotors();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }