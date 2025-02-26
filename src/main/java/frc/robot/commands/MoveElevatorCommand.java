package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorHeights;

public class MoveElevatorCommand extends Command {
    private final Elevator elevator;
    private final ElevatorHeights elevatorPosition;

    public MoveElevatorCommand(Elevator elevator, ElevatorHeights elevatorPosition){
        this.elevator = elevator;
        this.elevatorPosition = elevatorPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        elevator.moveElevator(elevatorPosition);
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