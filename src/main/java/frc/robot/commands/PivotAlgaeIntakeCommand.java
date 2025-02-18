package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intakes.AlgaeIntake;

public class PivotAlgaeIntakeCommand extends Command {
    private final AlgaeIntake algaeIntake;
    private final double velocity;

    public PivotAlgaeIntakeCommand(AlgaeIntake algaeIntake, double velocity){
        this.algaeIntake = algaeIntake;
        this.velocity = velocity;
        // addRequirements(coralIntake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        algaeIntake.pivotAlgaeIntake(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        algaeIntake.stopPivotAlgaeIntakeMotor();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }