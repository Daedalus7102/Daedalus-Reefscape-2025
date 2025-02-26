// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.Intakes.CoralIntakeConstants;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants.MergedAlgaeScorePositions;
import frc.robot.Constants.Intakes.CoralIntakeConstants.MergedCoralScorePositions;
import frc.robot.commands.AimbotCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GrabCoralFromSource;
import frc.robot.commands.MoveAlgaeCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorHeights;
import frc.robot.subsystems.Intakes.AlgaeIntake;
import frc.robot.subsystems.Intakes.CoralIntake;
import frc.robot.subsystems.Intakes.AlgaeIntake.AlgaeIntakeMode;
import frc.robot.subsystems.Intakes.CoralIntake.CoralIntakeMode;
import frc.robot.utils.TriggerButton;

public class RobotContainer {
  // Subsystems
  public Swerve swerve = new Swerve();
  public Elevator elevator = new Elevator();
  public CoralIntake coralIntake = new CoralIntake();
  public AlgaeIntake algaeIntake = new AlgaeIntake();

  // Controllers
  public static final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static final CommandPS5Controller operatorController = new CommandPS5Controller(1);
  private SendableChooser<Command> autoChooser;
  

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }

  private MergedCoralScorePositions sumPosition(MergedCoralScorePositions base, int sum) {
    int current;
    switch (base) {
      case HOME:
        current = 0;
        break;
      case L1:
        current = 1;
        break;
      case L2:
        current = 2;
        break;
      case L3:
        current = 3;
        break;
      case L4:
        current = 4;
        break;
      default:
        current = 0;
    }

    current += sum;
    
    if (current <= 0) {
      return MergedCoralScorePositions.HOME;
    } else if (current >= 4) {
      return MergedCoralScorePositions.L4;
    }

    if (current == 1) {
      return MergedCoralScorePositions.L1;
    } else if (current == 2) {
      return MergedCoralScorePositions.L2;
    } else {
      return MergedCoralScorePositions.L3;
    }
  }

  private void configureBindings() {/*
    // ----------- Driver Controller -----------
    swerve.setDefaultCommand(
      new DriveCommand(
        swerve,
        () -> (-driverController.getLeftY()),
        () -> (driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        SwerveConstants.chassisHighMaxOutput,
        false
      )
    );

    driverController.L1().whileTrue(
      new DriveCommand(
        swerve,
        () -> (-driverController.getLeftY()),
        () -> (driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        SwerveConstants.chassisLowMaxOutput,
        true
      )
    );*/

    // ------------ Driver Controller ------------
    driverController.triangle().whileTrue(new InstantCommand(() -> swerve.zeroHeading()));
    // driverController.square().whileTrue(new AimbotCommand(swerve, elevator, coralIntake, ElevatorPosition.L3, CoralIntakeMode.L2_AND_L3EJECT));
    // driverController.square().whileFalse(new MoveElevatorCommand(elevator, ElevatorPosition.HOME));

    // ----------- Operator Controller -----------
    // operatorController.square().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, MergedCoralScorePositions.L2));
    operatorController.triangle().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.HOME));
    operatorController.circle().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.INTAKE).alongWith(new InstantCommand(() -> coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeIntakeVleocity, true))));

    operatorController.cross().toggleOnTrue(new InstantCommand(() -> elevator.moveElevator(ElevatorHeights.L3)));
    operatorController.L1().whileTrue(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.FLOOR_INTAKE).alongWith(new InstantCommand(() -> algaeIntake.moveAlgaeIntakeMotors(-0.1, false))));
    operatorController.L1().whileFalse(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.HOME));
    operatorController.povLeft().toggleOnTrue(new InstantCommand(() -> coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity, false)));

    /*/ new controls
    TriggerButton l2Trigger = new TriggerButton(() -> (operatorController.getL2Axis() >= 0.4));
    l2Trigger
      .whileTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.READ_REEF_APRILTAG));
    l2Trigger.and(operatorController.povUp())
      .whileTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, sumPosition(, 1)));
    l2Trigger.and(operatorController.povDown())
      .whileTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, sumPosition(, 1)));
    l2Trigger.and(operatorController.triangle()).whileTrue(new EjectCoralCommand());
    l2Trigger.and(operatorController.cross()).whileTrue(new SuckCoralCommand());
    // l2Trigger.whileFalse(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.HOME));

    TriggerButton r2Trigger = new TriggerButton(() -> (operatorController.getR2Axis() >= 0.4));
    r2Trigger
      .whileTrue(new MoveAlgaeCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.READ_REEF_APRILTAG));
    r2Trigger.and(operatorController.povUp())
      .whileTrue(new MoveAlgaeCommand(elevator, coralIntake, algaeIntake, sumPosition(, 1)));
    r2Trigger.and(operatorController.povDown())
      .whileTrue(new MoveAlgaeCommand(elevator, coralIntake, algaeIntake, sumPosition(, 1)));
    r2Trigger.and(operatorController.triangle()).whileTrue(new EjectAlgaeCommand());
    r2Trigger.and(operatorController.cross()).whileTrue(new SuckAlgaeCommand());
*/
    /*
    operatorController.povDown().toggleOnTrue(new MoveElevatorCommand(elevator, ElevatorPosition.HOME));
    operatorController.povLeft().toggleOnTrue(new MoveElevatorCommand(elevator, ElevatorPosition.L2));
    driverController.povUp().toggleOnTrue(new MoveElevatorCommand(elevator, ElevatorPosition.READ_REEF_APRILTAG));
    operatorController.povRight().toggleOnTrue(new MoveElevatorCommand(elevator, ElevatorPosition.L4));

    operatorController.cross().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, ElevatorPosition.L1, CoralIntakeMode.L1_EJECT, operatorController.R2()));
    operatorController.circle().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, ElevatorPosition.L2, CoralIntakeMode.L2_AND_L3EJECT, operatorController.R2()));
    operatorController.triangle().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, ElevatorPosition.L3, CoralIntakeMode.L2_AND_L3EJECT, operatorController.R2()));
    operatorController.R1().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, ElevatorPosition.L4, CoralIntakeMode.L4_EJECT, operatorController.R2()));
    operatorController.L1().toggleOnTrue(new GrabCoralFromSource(elevator, coralIntake, operatorController.R2()));
 */

    // operatorController.cross().or
    // (operatorController.square()).or
    // (operatorController.triangle()).or
    // (operatorController.circle()).or
    // (operatorController.R1()).or
    // (operatorController.L1()).whileFalse(new CoralScoreCommand(coralIntake, CoralIntakeMode.HOME));

    // operatorController.L1().whileTrue(new AlgaeCommand(algaeIntake, AlgaeIntakeMode.FLOOR_INTAKE));
    // operatorController.R1().whileTrue(new AlgaeCommand(algaeIntake, AlgaeIntakeMode.PROCCESOR_EJECT));    
  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return autoChooser.getSelected();
  }

  public Swerve getSwerveSubsystem() {
    return swerve;
  }

  public Elevator getElevatorSubsystem() {
    return elevator;
  }
}