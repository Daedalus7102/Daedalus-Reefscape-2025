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
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.AlignRotationCommand;
import frc.robot.commands.CoralScoreCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.PivotAlgaeIntakeCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intakes.AlgaeIntake;
import frc.robot.subsystems.Intakes.AlgaeIntake.AlgaeIntakeMode;
import frc.robot.subsystems.Intakes.CoralIntake;
import frc.robot.subsystems.Intakes.CoralIntake.CoralIntakeMode;

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

  private void configureBindings() {
    // ----------- Driver Controller -----------
    swerve.setDefaultCommand(
      new DriveCommand(
        swerve,
        () -> (-driverController.getLeftY()),
        () -> (driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        false,
        SwerveConstants.chassisHighMaxOutput
      )
    );

    driverController.L1().whileTrue(
      new DriveCommand(
        swerve,
        () -> (-driverController.getLeftY()),
        () -> (driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        true,
        SwerveConstants.chassisLowMaxOutput
      )
    );

    // ------------ Driver Controller ------------
    driverController.triangle().whileTrue(new InstantCommand(() -> swerve.zeroHeading()));
    driverController.square().whileTrue(new AlignRotationCommand(swerve));

    // ----------- Operator Controller -----------
    operatorController.povUp().whileTrue(new MoveElevatorCommand(elevator, ElevatorPosition.L1));
    operatorController.povDown().whileTrue(new MoveElevatorCommand(elevator, ElevatorPosition.HOME));

    operatorController.triangle().whileTrue(new CoralScoreCommand(coralIntake, CoralIntakeMode.EJECT));
    operatorController.cross().whileTrue(new CoralScoreCommand(coralIntake, CoralIntakeMode.INTAKE));

    operatorController.L1().whileTrue(new AlgaeCommand(algaeIntake, AlgaeIntakeMode.INTAKE));
    operatorController.R1().whileTrue(new AlgaeCommand(algaeIntake, AlgaeIntakeMode.EJECT));
  
    operatorController.L2().whileTrue(new PivotAlgaeIntakeCommand(algaeIntake, 0.1));
    operatorController.R2().whileTrue(new PivotAlgaeIntakeCommand(algaeIntake, -0.1));
  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return autoChooser.getSelected();
  }

  public Swerve getChasisSubsystem() {
    return swerve;
  }

  public Elevator getElevatorSubsystem() {
    return elevator;
  }
}