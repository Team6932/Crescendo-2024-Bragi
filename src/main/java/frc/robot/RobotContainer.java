// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PieceConstants;
//import frc.robot.commands.TestDrive;
//import frc.robot.commands.ClimbSystemCommands.ClimbCommand;
import frc.robot.commands.DriveSystemCommands.ResetHeadingCommand;
import frc.robot.commands.IntakeSystemCommands.IntakeCommand;
import frc.robot.commands.IntakeSystemCommands.IntakeInCommand;
import frc.robot.commands.IntakeSystemCommands.IntakeOutCommand;
import frc.robot.commands.IntakeSystemCommands.ManualIntakeCommand;
import frc.robot.commands.IntakeSystemCommands.ResetIntakeCommand;
import frc.robot.commands.IntakeSystemCommands.ManualIntakeMoveCommand;
//import frc.robot.commands.LimelightCommands.LimelightDrive;
//import frc.robot.commands.LimelightCommands.setAprilTagCommand;
//import frc.robot.commands.LimelightCommands.setCameraCommand;
//import frc.robot.commands.LimelightCommands.setNeuralNetworkCommand;
//import frc.robot.commands.LimelightCommands.setVisionModeCommand;
import frc.robot.commands.ShootingSystemCommands.ShootCommand;
import frc.robot.commands.ShootingSystemCommands.SpeakerCommand;
import frc.robot.commands.UNUSEDFromYAGSL.AbsoluteDrive;
//import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeMoveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShootSubsystem shootSubsystem = new ShootSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeMoveSubsystem intakeMoveSubsystem = new IntakeMoveSubsystem();
  //private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  PS4Controller driveController = new PS4Controller(0);
  PS4Controller pieceController = new PS4Controller(1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // register named commands for PathPlanner
    NamedCommands.registerCommand("ramp", 
      new ShootCommand(shootSubsystem, PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower).withTimeout(1));
    
    NamedCommands.registerCommand("shoot", 
      new ShootCommand(shootSubsystem, PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower).withTimeout(0.5));

    NamedCommands.registerCommand("feed", 
      new ManualIntakeCommand(intakeSubsystem, PieceConstants.leftUpSpeakerFeedPower, -PieceConstants.rightDownSpeakerFeedPower)
      .withTimeout(0.5));

    NamedCommands.registerCommand("speaker", 
      new SpeakerCommand(intakeSubsystem, shootSubsystem, 
      PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower, 
      PieceConstants.leftUpSpeakerFeedPower, -PieceConstants.rightDownSpeakerFeedPower)
      .withTimeout(2));

    NamedCommands.registerCommand("stopArm", 
      new ParallelCommandGroup(
        new ShootCommand(shootSubsystem, 0, 0), 
        new ManualIntakeCommand(intakeSubsystem, 0, 0), 
        new IntakeOutCommand(intakeMoveSubsystem, 0, 0, 0, 0))
        .withTimeout(0.1));//, 
        //new ClimbCommand(climbSubsystem, 0)));

    NamedCommands.registerCommand("intakeOut", 
      new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
      PieceConstants.intakeOutP, 
      PieceConstants.intakeOutI,
      PieceConstants.intakeOutD)
      .withTimeout(2));

    NamedCommands.registerCommand("intakeIn", 
      new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP,
      PieceConstants.intakeInI,
      PieceConstants.intakeInD)
      .withTimeout(2));

    NamedCommands.registerCommand("intake", 
      new ManualIntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, PieceConstants.rightDownIntakePower)
      .withTimeout(2));

    NamedCommands.registerCommand("autoIntake", 
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new IntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, PieceConstants.rightDownIntakePower)
          .withTimeout(3),
          new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle,
            PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD)), 
        new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
          PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD))
        .withTimeout(3));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
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
        () -> -driveController.getRightX(),
        () -> -driveController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -OperatorConstants.drivePowerPercent * 
          MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -OperatorConstants.drivePowerPercent *
          MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -OperatorConstants.turnPowerPercent * 
          driveController.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -OperatorConstants.drivePowerPercent * 
          MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -OperatorConstants.drivePowerPercent * 
          MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -OperatorConstants.turnPowerPercent * 
          driveController.getRightX());

    drivebase.setDefaultCommand( // if isSimulation = not true, angular velocity; else, direct angle sim
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings() {

    Trigger stopAllArm = new Trigger(() -> driveController.getPSButton()).
      or(() -> pieceController.getPSButton()); // is PS4 Button on piece or drive, stop everything other than movement 

    Trigger resetHeading = new Trigger(() -> driveController.getOptionsButton()); // if Options on drive, reset heading
    Trigger resetIntake = new Trigger(() -> pieceController.getOptionsButton()); // if Options on piece, reset encoder value

    Trigger halfSpeed = new Trigger(() -> driveController.getL1Button()); // if L1 on drive, half speed
		Trigger halfTurn = new Trigger (() -> pieceController.getCircleButton()); // if Circle on piece, half turn speed	
    Trigger halfMovement = new Trigger(halfSpeed).and(halfTurn); // if L1 on drive AND Circle on piece, half movement

		Trigger speaker = new Trigger(() -> pieceController.getR1Button()); // if R1 on piece, speaker shoot
    Trigger amp = new Trigger(() -> pieceController.getPOV() == 90); // if D-Pad right on piece, amp shoot
    Trigger intake = new Trigger(() -> pieceController.getL1Button()); // if L1 on piece, intake a piece 

    Trigger autoIntakeOut = new Trigger (() -> pieceController.getPOV() == 0); // if D-Pad up on piece, auto move intake out
    Trigger autoIntakeIn = new Trigger (() -> pieceController.getPOV() == 180); // if D-Pad down on piece, auto move intake in

    Trigger fullAutoIntake = new Trigger(() -> pieceController.getPOV() == 270); // if D-Pad left on piece, full auto intake

    Trigger manualIntake = new Trigger(() -> pieceController.getTriangleButton()); // if Triangle on piece, turn on intake
    Trigger manualIntakeOut = new Trigger(() -> pieceController.getSquareButton()); // if Square on piece, manually move intake out
    Trigger manualIntakeIn = new Trigger(() -> pieceController.getCrossButton()); // if Cross on piece, manually move intake in

    Trigger climbUp = new Trigger(() -> driveController.getTriangleButton()); // if Triangle on drive, move climber up
    Trigger climbDown = new Trigger(() -> driveController.getCrossButton());// if Cross on drive, move climber down

    Trigger fullClimb = new Trigger(() -> driveController.getCircleButton()); // if Circle on drive, full send climber

    Trigger aprilTag = new Trigger(() -> driveController.getPOV() == 90); // if D-Pad right on drive, Limelight to April Tag mode
    Trigger neuralNetwork = new Trigger(() -> driveController.getPOV() == 270); // if D-Pad left on drive, Limelight to neural mode

    Trigger visionMode = new Trigger(() -> driveController.getPOV() == 180); // if D-Pad down on drive, Limelight to vision process
    Trigger driveMode = new Trigger(() -> driveController.getPOV() == 0); // if D-Pad up on drive, Limelight to driver camera

    // simultaneously push game piece into shooter and shoot for speaker
    speaker.whileTrue(new SpeakerCommand(intakeSubsystem, shootSubsystem, 
      PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower, 
      PieceConstants.leftUpSpeakerFeedPower, -PieceConstants.rightDownSpeakerFeedPower));
		/*speaker.whileTrue(new ShootCommand(shootSubsystem, PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower));
    //shoot.onTrue(new IntakeCommand(intakeSubsystem, -0.1, -0.1).withTimeout(0.3));
    speaker.whileTrue(new WaitCommand(1.2). andThen(new IntakeCommand(
      intakeSubsystem, -PieceConstants.leftUpSpeakerFeedPower, -PieceConstants.rightDownSpeakerFeedPower))); */

    // simultaneously push game piece into shooter and shoot for amp
    amp.whileTrue(new ShootCommand(shootSubsystem, PieceConstants.leftAmpPower, PieceConstants.rightAmpPower));
    amp.whileTrue(new WaitCommand(0.7). andThen(new ManualIntakeCommand(
      intakeSubsystem, PieceConstants.leftUpAmpFeedPower, PieceConstants.rightDownAmpFeedPower)));

    // automatically move intake out and grab game pieces and then move intake in
    /*intake.whileTrue(new ManualIntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, PieceConstants.rightDownIntakePower));
    intake.onTrue(new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
      PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD));
    intake.onFalse(new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD));*/
    intake.onTrue(new ParallelCommandGroup(
      new IntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, PieceConstants.rightDownIntakePower), 
      new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
        PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD)));
    intake.onFalse(new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD));

    // automatically move intake in/out
    autoIntakeOut.onTrue(new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
      PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD));
    autoIntakeIn.onTrue(new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD)); 

    // manully move intake in/out and manually grab pieces
    manualIntakeOut.whileTrue(new ManualIntakeMoveCommand(intakeMoveSubsystem, -PieceConstants.intakeMovePower));
    manualIntakeIn.whileTrue(new ManualIntakeMoveCommand(intakeMoveSubsystem, PieceConstants.intakeMovePower));
    manualIntake.whileTrue(new ManualIntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, 0.7*PieceConstants.rightDownIntakePower));

    // manually move climb mechanism up and down
    /*climbUp.whileTrue(new ClimbCommand(climbSubsystem, PieceConstants.climbPower));
    climbDown.whileTrue(new ClimbCommand(climbSubsystem, -PieceConstants.climbPower));
    fullClimb.whileTrue(new ClimbCommand(climbSubsystem, -PieceConstants.fullClimbPower));*/

    // reset heading/intake encoder
    resetHeading.onTrue(new ResetHeadingCommand(drivebase));
    resetIntake.onTrue(new ResetIntakeCommand(intakeMoveSubsystem));

    // fully automatic intake 
    fullAutoIntake.onTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new IntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, PieceConstants.rightDownIntakePower)
        .withTimeout(3),
        new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle,
            PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD)), 
      new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
        PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD))
      .withTimeout(3));

    // Limelight modes
    /*aprilTag.toggleOnTrue(new setAprilTagCommand(limelightSubsystem));
    neuralNetwork.toggleOnTrue(new setNeuralNetworkCommand(limelightSubsystem));
    visionMode.toggleOnTrue(new setVisionModeCommand(limelightSubsystem, LimelightConstants.visionProcessModeId));
    driveMode.toggleOnTrue(new setVisionModeCommand(limelightSubsystem, LimelightConstants.cameraModeId));*/

    // half speed/drive (probably a better way to code this)
    Command halfSpeedCommand = drivebase.driveCommand(
      () -> -OperatorConstants.drivePowerPercent * 0.5 *
        MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -OperatorConstants.drivePowerPercent * 0.5 *
        MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -OperatorConstants.turnPowerPercent * 
        driveController.getRightX());
    Command halfTurnCommand = drivebase.driveCommand(
      () -> -OperatorConstants.drivePowerPercent * 
        MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -OperatorConstants.drivePowerPercent * 
        MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -OperatorConstants.turnPowerPercent * 0.5 *
        driveController.getRightX());
    Command halfMovementCommand = drivebase.driveCommand(
      () -> -OperatorConstants.drivePowerPercent * 0.5 * 
        MathUtil.applyDeadband(driveController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -OperatorConstants.drivePowerPercent * 0.5 * 
        MathUtil.applyDeadband(driveController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -OperatorConstants.turnPowerPercent * 0.5 *
        driveController.getRightX());
    
    halfSpeed.whileTrue(halfSpeedCommand);
    halfTurn.whileTrue(halfTurnCommand);
    halfMovement.whileTrue(halfMovementCommand);

    // shut down everything other than movement 
    stopAllArm.whileTrue(new ParallelCommandGroup(
      new ShootCommand(shootSubsystem, 0, 0), 
      new ManualIntakeCommand(intakeSubsystem, 0, 0), 
      new IntakeOutCommand(intakeMoveSubsystem, 0, 0, 0, 0)));//, 
      //new ClimbCommand(climbSubsystem, 0)));        
    stopAllArm.onTrue(new ParallelCommandGroup(
      new ShootCommand(shootSubsystem, 0, 0), 
      new ManualIntakeCommand(intakeSubsystem, 0, 0), 
      new IntakeOutCommand(intakeMoveSubsystem, 0, 0, 0, 0)));//, 
      //new ClimbCommand(climbSubsystem, 0))).notifyAll();    
          


    /*Trigger moveTest = new Trigger(() -> pieceController.getL3Button());
    moveTest.whileTrue(new TestDrive(limelightSubsystem, drivebase));

    Trigger testing = new Trigger(() -> pieceController.getR3Button());
    testing.whileTrue(drivebase.doubleDriveCommand(5, 5, 0));*/
  
///////////////////// TESTING ////////////////////
    /* Trigger moveTest = new Trigger(() -> driveController.getSquareButton()); 
    moveTest.onTrue(drivebase.driveToPose(
      new Pose2d(new Translation2d(2, 1), Rotation2d.fromDegrees(90))
    ));

    Trigger limelightTest = new Trigger(() -> driveController.getR1Button());
    limelightTest.onTrue(new LimelightDrive(limelightSubsystem, drivebase)); */

    /*driveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driveController.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); */
    // driveController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /** ////////////////////USE THIS FOR PATHPLANNER ///////////////////////////////////////////////////////////////////////////////////////
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("testAuto");
  } */

  /*public Command getAutonomousCommand() {
    return (drivebase.driveToPose(
      new Pose2d(new Translation2d(5, 0), Rotation2d.fromDegrees(0.0))
    )); 
  } */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
