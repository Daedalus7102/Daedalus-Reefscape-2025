package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Utilities.PositionEnums.AlgaeScorePositions;
import frc.robot.Utilities.PositionEnums.CoralScorePositions;

public class MoveElevatorCommand extends Command {
    private final Elevator elevator;
    private AlgaeScorePositions algaeElevatorPosition = null;
    private CoralScorePositions coralElevatorPosition = null;

    public MoveElevatorCommand(Elevator elevator, CoralScorePositions elevatorPosition) {
        this.elevator = elevator;
        this.coralElevatorPosition = elevatorPosition;
        addRequirements(elevator);
    }

    public MoveElevatorCommand(Elevator elevator, AlgaeScorePositions elevatorPosition) {
        this.elevator = elevator;
        this.algaeElevatorPosition = elevatorPosition;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if (algaeElevatorPosition != null) {
            elevator.moveElevator(algaeElevatorPosition);
        } else {
            elevator.moveElevator(coralElevatorPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotors();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }