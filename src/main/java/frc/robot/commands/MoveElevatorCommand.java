package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorPosition;

public class MoveElevatorCommand extends Command {
    private final Elevator elevator;
    private final ElevatorPosition elevatorPosition;
    private final double velocity;

    public MoveElevatorCommand(Elevator elevator, ElevatorPosition elevatorPosition, double velocity){
        this.elevator = elevator;
        this.elevatorPosition = elevatorPosition;
        this.velocity = velocity;
        // addRequirements(elevator);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        elevator.moveElevator(velocity, true);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevatorMotors();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }