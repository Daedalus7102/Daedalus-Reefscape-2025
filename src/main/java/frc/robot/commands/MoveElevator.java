package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorPosition;

public class MoveElevator extends Command {
    private final Elevator elevator;
    private final ElevatorPosition elevatorPosition;

    public MoveElevator(Elevator elevator, ElevatorPosition elevatorPosition){
        this.elevator = elevator;
        this.elevatorPosition = elevatorPosition;
        // addRequirements(elevator);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        elevator.moveElevatorMotorsUp();
        SmartDashboard.putNumber("Encoder value", elevator.getElevatorPosition());
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