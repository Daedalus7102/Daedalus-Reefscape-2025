package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorHeights;
import frc.robot.subsystems.Intakes.CoralIntake;
import frc.robot.subsystems.Intakes.CoralIntake.CoralIntakeMode;

public class GrabCoralFromSource extends Command{
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private Trigger shoot;
  boolean isFinished;

  Timer timer = new Timer();

  public GrabCoralFromSource(Elevator elevator, CoralIntake coralIntake, Trigger shoot) {
      this.elevator = elevator;
      this.coralIntake = coralIntake;
      this.shoot = shoot;
      addRequirements(elevator);
    }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.moveElevator(ElevatorHeights.PICKUP);
    coralIntake.moveCoralIntake(CoralIntakeMode.INTAKE_PICKUP);
    if(shoot.getAsBoolean()) {
        coralIntake.moveCoralIntakeMotors(-0.1, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.moveCoralIntakeMotors(0.05, false);
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}