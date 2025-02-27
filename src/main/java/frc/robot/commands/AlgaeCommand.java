package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntakeMode;

public class AlgaeCommand extends Command {
    private final AlgaeIntake algaeIntake;

    public AlgaeCommand(AlgaeIntake algaeIntake, AlgaeIntakeMode algaeIntakeMode){
        this.algaeIntake = algaeIntake;
        addRequirements(algaeIntake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        // ADD PIVOT MOTOR COMMAND
        algaeIntake.moveAlgaeIntakeMotors(-0.1, true);
    }

    @Override
    public void end(boolean interrupted) {
        algaeIntake.stopPivotMotor();
        algaeIntake.stopIntakeMotors();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }