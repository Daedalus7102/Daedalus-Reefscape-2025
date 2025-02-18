package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorPosition;

public class MoveElevatorCommand extends Command {
    private final Elevator elevator;
    private final ElevatorPosition elevatorPosition;

    public MoveElevatorCommand(Elevator elevator, ElevatorPosition elevatorPosition){
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