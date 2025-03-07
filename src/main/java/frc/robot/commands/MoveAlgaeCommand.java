package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants.MergedAlgaeScorePositions;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorHeights;
import frc.robot.subsystems.Intakes.AlgaeIntake;
import frc.robot.subsystems.Intakes.AlgaeIntake.AlgaeIntakeMode;
import frc.robot.subsystems.Intakes.CoralIntake;
import frc.robot.subsystems.Intakes.CoralIntake.CoralIntakeMode;

public class MoveAlgaeCommand extends Command{
  private final Elevator elevator;
  private final AlgaeIntake algaeIntake;
  private final CoralIntake coralIntake;
  private final MergedAlgaeScorePositions mergedAlgaeScorePositions;

  public MoveAlgaeCommand(Elevator elevator, AlgaeIntake algaeIntake, CoralIntake coralIntake, MergedAlgaeScorePositions mergedAlgaeScorePositions) {
      this.elevator = elevator;
      this.algaeIntake = algaeIntake;
      this.coralIntake = coralIntake;
      this.mergedAlgaeScorePositions = mergedAlgaeScorePositions;
      addRequirements(elevator, algaeIntake, coralIntake);
    }
  
  @Override
  public void initialize() {
    coralIntake.moveCoralIntake(CoralIntakeMode.HOME, null);
  }

  @Override
  public void execute() {
    coralIntake.moveCoralIntake(CoralIntakeMode.HOME, null);
    switch (mergedAlgaeScorePositions) {
      case FLOOR_INTAKE:
        elevator.moveElevator(ElevatorHeights.FLOOR_ALGAE_INTAKE);
        algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.FLOOR_INTAKE);
        algaeIntake.moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeIntakeVelocity, false);
        break;

      case PROCCESOR_EJECT:
        elevator.moveElevator(ElevatorHeights.PROCESSOR_EJECT);
        algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.PROCCESOR_EJECT);
        break;

      case BETWEEN_L2_AND_L3Position:
        elevator.moveElevator(ElevatorHeights.BETWEEN_L2_AND_L3);
        algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position);
        algaeIntake.moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeIntakeVelocity, true);
        break;
      case BETWEEN_L3_AND_L4Position:
        elevator.moveElevator(ElevatorHeights.BETWEEN_L3_AND_L4);
        algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.BETWEEN_L2_AND_L3_OR_L3_AND_L4_Position);
        algaeIntake.moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeIntakeVelocity, true);
        break;
      case HOME:
        algaeIntake.moveAlgaeIntakeMotors(0, true);
        elevator.moveElevator(ElevatorHeights.HOME);
        algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.HOME_WITH_ALGAE);
        break;
      case NET_EJECTPosition:
        elevator.moveElevator(ElevatorHeights.NET);
        algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.NET_EJECT);
        break;
    }
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