// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive.Swerve;

public class RobotContainer {

  // Subsystems
  public Swerve swerve = new Swerve();

  // Controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(0);
  public static final CommandPS4Controller operatorController = new CommandPS4Controller(1);
  // private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);

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
        false
      )
    );

    driverController.L1().whileTrue(
      new DriveCommand(
        swerve,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (driverController.getRightX()),
        true
      )
    );

    // ------------ Driver Controller ------------
    driverController.triangle().whileTrue(new InstantCommand(() -> swerve.zeroHeading()));

    // ----------- Operator Controller -----------
  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return null; // autoChooser.getSelected();
  }

  public Swerve getChasisSubsystem() {
    return swerve;
  }
}