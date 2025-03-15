package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberMode;
import frc.robot.subsystems.Climbr;


public class ClimbCommand extends Command{
  private final Climbr climber;
  private final ClimberMode climberMode;

  public ClimbCommand(Climbr climber, ClimberMode climberMode) {
      this.climber = climber;
      this.climberMode = climberMode;
      addRequirements(climber);
    }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    switch (climberMode) {
      case HOME:
          climber.moveClimber(ClimberMode.HOME);
        break;

      case CLIMB:
          climber.moveClimber(ClimberMode.CLIMB);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}