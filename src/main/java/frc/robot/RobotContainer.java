// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeMoveCommand;
import frc.robot.commands.SimpleIntakeMoveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.SimpleIntakeMoveSubsystem;
import frc.robot.subsystems.IntakeMoveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShootSubsystem shootSubsystem = new ShootSubsystem();
  private final SimpleIntakeMoveSubsystem simpleIntakeMoveSubsystem = new SimpleIntakeMoveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeMoveSubsystem intakeMoveSubsystem = new IntakeMoveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  PS4Controller driveController = new PS4Controller(0);
  PS4Controller pieceController = new PS4Controller(1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
      () -> -MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -MathUtil.applyDeadband(driveController.getRightX(),OperatorConstants.RIGHT_X_DEADBAND),
      () -> -MathUtil.applyDeadband(driveController.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND));
    
      // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -pieceController.getRightX(),
        () -> -pieceController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -pieceController.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -pieceController.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    /* 
     *  drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
     */
  }

  private void configureBindings() {

    Trigger halfSpeed = new Trigger(() -> driveController.getL1Button()); // if L1 on drive, half speed
		Trigger halfTurn = new Trigger (() -> pieceController.getCircleButton()); // if Circle on piece, half turn speed	
		Trigger resetHeading = new Trigger(() -> driveController.getOptionsButton()); // if Options on drive, reset heading

		Trigger shoot = new Trigger(() -> pieceController.getR1Button()); // if R1 on piece, shoot
    Trigger intake = new Trigger(() -> pieceController.getL1Button()); // if L1 on piece, activate the intake 
    Trigger manualIntake = new Trigger(() -> pieceController.getTriangleButton()); // if Triangle on piece, turn on chains
    Trigger simpleIntakeOut = new Trigger(() -> pieceController.getSquareButton()); // if Square on piece, move intake out
    Trigger simpleIntakeIn = new Trigger(() -> pieceController.getCrossButton()); // if Cross on piece, move intake in

		shoot.whileTrue(new ShootCommand(shootSubsystem, 0.5, 0.5));
    shoot.whileTrue(new IntakeCommand(intakeSubsystem, 0.0, 0.0));

    simpleIntakeOut.whileTrue(new SimpleIntakeMoveCommand(simpleIntakeMoveSubsystem, 0.5));
    simpleIntakeIn.whileTrue(new SimpleIntakeMoveCommand(simpleIntakeMoveSubsystem, -0.5));

    intake.whileTrue(new IntakeCommand(intakeSubsystem, 0.5, 0.5));
    intake.whileTrue(new IntakeMoveCommand(intakeMoveSubsystem, 50));

    manualIntake.whileTrue(new IntakeCommand(intakeSubsystem, -0.5, -0.5));

    /*driveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driveController.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); */
    // driveController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
