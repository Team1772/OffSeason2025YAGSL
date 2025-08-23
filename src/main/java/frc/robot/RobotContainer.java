// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox;
  private final SwerveSubsystem drivebase;
  private final IntakeSubsystem intake;


  SwerveInputStream driveAngularVelocity;
  SwerveInputStream driveDirectAngle;
  SwerveInputStream driveRobotOriented;
  SwerveInputStream driveAngularVelocityKeyboard;
  SwerveInputStream driveDirectAngleKeyboard;

  Command driveFieldOrientedDirectAngle;
  Command driveFieldOrientedAnglularVelocity;
  Command driveRobotOrientedAngularVelocity;
  Command driveSetpointGen;
  Command driveFieldOrientedDirectAngleKeyboard;
  Command driveFieldOrientedAnglularVelocityKeyboard;
  Command driveSetpointGenKeyboard;

   public RobotContainer() {
    driverXbox = new CommandXboxController(0);
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    intake = new IntakeSubsystem();
    
    //Configures swerve input streams
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

    //Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
    driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

    //Clone's the angular velocity input stream and converts it to a robotRelative input stream.
    driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true).allianceRelativeControl(false);


    driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

    // Derive the heading axis with math!
    driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
      () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(0));

    //Configure commands
    driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    //Configure bindings
    configureBindings();
  }

  private void configureBindings() {
    configureDefaultBindings();
    configureSimulationBindings();
    configureTestBindings();
  }

  private void configureDefaultBindings() {
    if (!RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (!DriverStation.isTest()) {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

    driverXbox.x().whileTrue(intake.rodar());
  }

  private void configureSimulationBindings() {
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)
          ));
    }
  }

  private void configureTestBindings() {
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides default drive command!
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
