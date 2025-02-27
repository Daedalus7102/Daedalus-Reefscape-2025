package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.Elevator;
import frc.robot.Utilities.PositionEnums.AlgaeScorePositions;
import frc.robot.Utilities.PositionEnums.CoralScorePositions;


public class ScoreCoralCommand extends Command{
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final CoralScorePositions scorePositions;

  public ScoreCoralCommand(Elevator elevator, CoralIntake coralIntake, AlgaeIntake algaeIntake, CoralScorePositions scorePositions) {
      this.elevator = elevator;
      this.coralIntake = coralIntake;
      this.algaeIntake = algaeIntake;
      this.scorePositions = scorePositions;
      addRequirements(elevator);
    }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaeIntake.moveAlgaeIntake(AlgaeScorePositions.HOME);
    elevator.moveElevator(scorePositions);
    coralIntake.moveCoralIntake(scorePositions);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopMotors();
    coralIntake.stopPivotMotor();
    coralIntake.stopIntakeMotors();
    algaeIntake.stopPivotMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
