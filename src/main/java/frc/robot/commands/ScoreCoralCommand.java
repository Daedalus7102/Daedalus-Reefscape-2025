package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intakes.CoralIntakeConstants;
import frc.robot.Constants.Intakes.CoralIntakeConstants.MergedCoralScorePositions;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorHeights;
import frc.robot.subsystems.Intakes.CoralIntake;
import frc.robot.subsystems.Intakes.CoralIntake.CoralIntakeMode;
import frc.robot.subsystems.Intakes.AlgaeIntake;
import frc.robot.subsystems.Intakes.AlgaeIntake.AlgaeIntakeMode;


public class ScoreCoralCommand extends Command{
  private final Elevator elevator;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final MergedCoralScorePositions mergedCoralScorePositions;
  private Supplier<Double> joyStickSupplier;

  public ScoreCoralCommand(Elevator elevator, CoralIntake coralIntake, AlgaeIntake algaeIntake, MergedCoralScorePositions mergedCoralScorePositions, Supplier<Double> joyStickSupplier) {
      this.elevator = elevator;
      this.coralIntake = coralIntake;
      this.algaeIntake = algaeIntake;
      this.mergedCoralScorePositions = mergedCoralScorePositions;
      this.joyStickSupplier = joyStickSupplier;
      addRequirements(elevator);
    }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaeIntake.moveAlgaeIntake(AlgaeIntakeMode.HOME);

    double angleCorrection = 0;
    if(Math.abs(joyStickSupplier.get()) > 0.1) {
        angleCorrection = -joyStickSupplier.get() * 10;
    }

    switch (mergedCoralScorePositions) {
      case READ_REEF_APRILTAG:
        break;

      case HOME:
        coralIntake.moveCoralIntakeMotors(0, true);
        elevator.moveElevator(ElevatorHeights.HOME);
        coralIntake.moveCoralIntake(CoralIntakeMode.HOME, 0);
        break;

      case INTAKE:
        coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeIntakeVleocity, true);
        elevator.moveElevator(ElevatorHeights.PICKUP);
        coralIntake.moveCoralIntake(CoralIntakeMode.INTAKE_PICKUP, angleCorrection);
        break;

      case L1:
        elevator.moveElevator(ElevatorHeights.L1);
        coralIntake.moveCoralIntake(CoralIntakeMode.L1_EJECT, angleCorrection);
        break;

      case L2:
        elevator.moveElevator(ElevatorHeights.L2);
        coralIntake.moveCoralIntake(CoralIntakeMode.L2_AND_L3EJECT, angleCorrection);
        break;
      
      case L3:
        elevator.moveElevator(ElevatorHeights.L3);
        coralIntake.moveCoralIntake(CoralIntakeMode.L2_AND_L3EJECT, angleCorrection);
        break;

      case L4:
        elevator.moveElevator(ElevatorHeights.L4);
        coralIntake.moveCoralIntake(CoralIntakeMode.L4_EJECT, angleCorrection);
        break;
    }
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