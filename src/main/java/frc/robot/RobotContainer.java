// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants;
import frc.robot.Constants.Intakes.CoralIntakeConstants;
import frc.robot.Constants.Intakes.AlgaeIntakeConstants.MergedAlgaeScorePositions;
import frc.robot.Constants.Intakes.CoralIntakeConstants.MergedCoralScorePositions;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AutoPositionCoralCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MoveAlgaeCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intakes.AlgaeIntake;
import frc.robot.subsystems.Intakes.CoralIntake;

public class RobotContainer {
  // Subsystems
  public Swerve swerve = new Swerve();
  public Elevator elevator = new Elevator();
  private CoralIntake coralIntake = new CoralIntake();
  private AlgaeIntake algaeIntake = new AlgaeIntake();

  // Controllers
  public static final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static final CommandPS5Controller operatorController = new CommandPS5Controller(1);
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("SCORE_L1", new AutoPositionCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.L1).withTimeout(2));
    NamedCommands.registerCommand("EJECT_FOR_L1", new InstantCommand(() -> coralIntake.moveCoralIntakeMotorsForL1()).withTimeout(2));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    // ----------- Driver Controller -----------
    swerve.setDefaultCommand(
      new DriveCommand(
        swerve,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (driverController.getRightX()),
        SwerveConstants.chassisHighMaxOutput,
        false
      )
    );

    driverController.L1().whileTrue(
      new DriveCommand(
        swerve,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (driverController.getRightX()),
        0.5,
        true
      )
    );

    // ------------ Driver Controller ------------
    driverController.triangle().whileTrue(new InstantCommand(() -> swerve.zeroHeading()));
    // driverController.square().whileTrue(new AimbotCommand(swerve, elevator, coralIntake, false));
    // driverController.circle().whileTrue(new AimbotCommand(swerve, elevator, coralIntake, true));
    // driverController.circle().whileFalse(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.HOME, () -> operatorController.getRightY()));
    driverController.options().whileTrue(new InstantCommand(() -> elevator.resetMotorEncoders()));

    // ----------- Operator Controller -----------
    operatorController.povDown().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.L1, () -> operatorController.getRightY()));
    operatorController.povLeft().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.L2, () -> operatorController.getRightY()));
    operatorController.povUp().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.L3, () -> operatorController.getRightY()));
    operatorController.L1().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.L4, () -> operatorController.getRightY()));
    operatorController.povRight().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.INTAKE, () -> operatorController.getRightY())).whileFalse(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.HOME, () -> operatorController.getRightY()));

    operatorController.square().toggleOnTrue(new ScoreCoralCommand(elevator, coralIntake, algaeIntake, MergedCoralScorePositions.HOME, () -> operatorController.getRightY()));
    operatorController.touchpad().toggleOnTrue(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.HOME));

    operatorController.cross().toggleOnTrue(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.FLOOR_INTAKE));
    operatorController.circle().toggleOnTrue(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.PROCCESOR_EJECT));
    operatorController.triangle().toggleOnTrue(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.BETWEEN_L2_AND_L3Position));
    operatorController.R1().toggleOnTrue(new MoveAlgaeCommand(elevator, algaeIntake, coralIntake, MergedAlgaeScorePositions.BETWEEN_L3_AND_L4Position));

    operatorController.R2().whileTrue(new InstantCommand(() -> coralIntake.moveCoralIntakeMotors(CoralIntakeConstants.coralIntakeEjectVelocity, false))).whileFalse(new InstantCommand(() -> coralIntake.stopIntakeMotors()));
    operatorController.R1().whileTrue(new InstantCommand(() -> coralIntake.moveCoralIntakeMotorsForL1())).whileFalse(new InstantCommand(() -> coralIntake.stopIntakeMotors()));
    operatorController.L2().whileTrue(new InstantCommand(() -> algaeIntake.moveAlgaeIntakeMotors(AlgaeIntakeConstants.algaeIntakeEjectVelocity, false))).whileFalse(new InstantCommand(() -> algaeIntake.stopIntakeMotors()));
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

  public AlgaeIntake getAlgaeIntakeSubsystem() {
    return algaeIntake;
  }

  public CoralIntake getcCoralIntake() {
    return coralIntake;
  }
}