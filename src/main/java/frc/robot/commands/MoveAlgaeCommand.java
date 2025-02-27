package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.Elevator;
import frc.robot.Utilities.PositionEnums.AlgaeScorePositions;
import frc.robot.Utilities.PositionEnums.CoralScorePositions;

public class MoveAlgaeCommand extends Command{
  private final Elevator elevator;
  private final AlgaeIntake algaeIntake;
  private final CoralIntake coralIntake;
  private final AlgaeScorePositions scorePosition;

  public MoveAlgaeCommand(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake, AlgaeScorePositions scorePosition) {
      this.elevator = elevator;
      this.algaeIntake = algaeIntake;
      this.coralIntake = coralIntake;
      this.scorePosition = scorePosition;
      addRequirements(elevator, algaeIntake, coralIntake);
    }
  
  @Override
  public void initialize() {
    coralIntake.moveCoralIntake(CoralScorePositions.HOME);
  }

  @Override
  public void execute() {
    

    coralIntake.moveCoralIntake(CoralScorePositions.HOME);
    elevator.moveElevator(scorePosition);
    algaeIntake.moveAlgaeIntake(scorePosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopMotors();
    algaeIntake.stopPivotMotor();
    algaeIntake.stopIntakeMotors();
    coralIntake.stopPivotMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
