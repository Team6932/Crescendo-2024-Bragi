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
import frc.robot.commands.DriveSystemCommands.ReverseResetHeadingCommand;
import frc.robot.commands.IntakeSystemCommands.IntakeCommand;
import frc.robot.commands.IntakeSystemCommands.IntakeInCommand;
import frc.robot.commands.IntakeSystemCommands.IntakeOutCommand;
import frc.robot.commands.IntakeSystemCommands.ManualIntakeCommand;
import frc.robot.commands.IntakeSystemCommands.ResetIntakeCommand;
import frc.robot.commands.IntakeSystemCommands.ManualIntakeMoveCommand;
import frc.robot.commands.ShootingSystemCommands.AmpCommand;
import frc.robot.commands.ShootingSystemCommands.PassCommand;
//import frc.robot.commands.LimelightCommands.LimelightDrive;
//import frc.robot.commands.LimelightCommands.setAprilTagCommand;
//import frc.robot.commands.LimelightCommands.setCameraCommand;
//import frc.robot.commands.LimelightCommands.setNeuralNetworkCommand;
//import frc.robot.commands.LimelightCommands.setVisionModeCommand;
import frc.robot.commands.ShootingSystemCommands.ShootCommand;
import frc.robot.commands.ShootingSystemCommands.SpeakerCommand;
import frc.robot.commands.ShootingSystemCommands.ImprovedCommands.ImprovedGeneralShootCommand;
import frc.robot.commands.ShootingSystemCommands.ImprovedCommands.ImprovedSpeakerShootCommand;
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

  /*
   * Define all the subsystems.
   */
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ShootSubsystem shootSubsystem = new ShootSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeMoveSubsystem intakeMoveSubsystem = new IntakeMoveSubsystem();
  //private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  /*
   * Define our two controllers for the drive team. 
   */
  PS4Controller driveController = new PS4Controller(0);
  PS4Controller pieceController = new PS4Controller(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    /*
     * Register all Named Commands for PathPlanner
     */
    NamedCommands.registerCommand("ramp", 
      new ShootCommand(shootSubsystem, PieceConstants.leftSpeakerPower * PieceConstants.signLeftShoot, 
        PieceConstants.rightSpeakerPower * PieceConstants.signRightShoot).withTimeout(1));
    
    NamedCommands.registerCommand("shoot", 
      new ShootCommand(shootSubsystem, PieceConstants.leftSpeakerPower * PieceConstants.signLeftShoot, 
        PieceConstants.rightSpeakerPower * PieceConstants.signRightShoot).withTimeout(0.5));

    NamedCommands.registerCommand("feed", 
      new ManualIntakeCommand(intakeSubsystem, PieceConstants.leftUpSpeakerFeedPower * PieceConstants.signLeftUpFeed, 
        PieceConstants.rightDownSpeakerFeedPower * PieceConstants.signRightDownFeed).withTimeout(0.5));

    NamedCommands.registerCommand("speaker", 
      new SpeakerCommand(intakeSubsystem, shootSubsystem, 
      PieceConstants.leftSpeakerPower * PieceConstants.signLeftShoot, 
      PieceConstants.rightSpeakerPower * PieceConstants.signRightShoot, 
      PieceConstants.leftUpSpeakerFeedPower * PieceConstants.signLeftUpFeed, 
      PieceConstants.rightDownSpeakerFeedPower * PieceConstants.signRightDownFeed).withTimeout(2));

    NamedCommands.registerCommand("stopArm", 
      new ParallelCommandGroup(
        new ShootCommand(shootSubsystem, 0, 0), 
        new ManualIntakeCommand(intakeSubsystem, 0, 0), 
        new IntakeOutCommand(intakeMoveSubsystem, 0, 0, 0, 0))
        .withTimeout(0.1));

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
      new ManualIntakeCommand(intakeSubsystem, PieceConstants.leftUpIntakePower * PieceConstants.signLeftUpIntake, 
        PieceConstants.rightDownIntakePower * PieceConstants.signRightDownIntake).withTimeout(2));

    NamedCommands.registerCommand("autoIntake", 
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new IntakeCommand(intakeSubsystem, PieceConstants.leftUpIntakePower * PieceConstants.signLeftUpIntake, 
            PieceConstants.rightDownIntakePower * PieceConstants.signRightDownIntake).withTimeout(3),
          new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle,
            PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD)), 
        new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
          PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD))
        .withTimeout(3));

    /*
     * Create an autoChooser, so we can select which autonomous routine we want to run. 
     */
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    /*
     * Configure our button bindings.
     * You can find it below. 
     */
    configureBindings();

    // Applies deadbands and inverts controls because joysticks are back-right positive while robot
    // controls are front-left positive; left stick controls translation; right stick controls the angular velocity of the robot
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

  /*
   * This is where we declare all of our button bindings and specify what they do. 
   * 
   * Use the Trigger class.
   * 
   * .whileTrue makes the command run continuously until the button is pressed.
   * An end condition is not strictly necesary. 
   * 
   * .onTrue makes the command run as soon as the button is pressed. 
   * An end condition is necessary. 
   */
  private void configureBindings() {

    // drive controller - PS4 Button, Options, Share, L1, R1
    // piece controller - PS4 Button, Options, L1, R1, L2, R2, D-Up, D-Down, D-Left, Triangle, Square, Cross


    /*Trigger testSpeaker = new Trigger(() -> driveController.getCircleButton());
    Trigger testAmp = new Trigger(() -> driveController.getCrossButton());
    Trigger testPass = new Trigger(() -> driveController.getTriangleButton());
    Trigger testImprovedSpeaker = new Trigger(() -> driveController.getSquareButton());
    Trigger testShootModeError = new Trigger(() -> driveController.getR2Button());

    testSpeaker.whileTrue(new ImprovedGeneralShootCommand(intakeSubsystem, shootSubsystem, "speaker"));
    testAmp.whileTrue(new ImprovedGeneralShootCommand(intakeSubsystem, shootSubsystem, "amp"));
    testPass.whileTrue(new ImprovedGeneralShootCommand(intakeSubsystem, shootSubsystem, "pass"));
    testImprovedSpeaker.whileTrue(new ImprovedSpeakerShootCommand(intakeSubsystem, shootSubsystem));
    testShootModeError.whileTrue(new ImprovedGeneralShootCommand(intakeSubsystem, shootSubsystem, "hello"));*/


    //Trigger climbUp = new Trigger(() -> driveController.getTriangleButton()); // if Triangle on drive, move climber up
    //Trigger climbDown = new Trigger(() -> driveController.getCrossButton());// if Cross on drive, move climber down
    //Trigger fullClimb = new Trigger(() -> driveController.getCircleButton()); // if Circle on drive, full send climber

    Trigger aprilTag = new Trigger(() -> driveController.getPOV() == 90); // if D-Pad right on drive, Limelight to April Tag mode
    Trigger neuralNetwork = new Trigger(() -> driveController.getPOV() == 270); // if D-Pad left on drive, Limelight to neural mode

    Trigger visionMode = new Trigger(() -> driveController.getPOV() == 180); // if D-Pad down on drive, Limelight to vision process
    Trigger driveMode = new Trigger(() -> driveController.getPOV() == 0); // if D-Pad up on drive, Limelight to driver camera

    // if R1 on piece, shoot for the speaker 
    Trigger speaker = new Trigger(() -> pieceController.getR1Button()); 
    speaker.whileTrue(new SpeakerCommand(intakeSubsystem, shootSubsystem, 
      PieceConstants.leftSpeakerPower * PieceConstants.signLeftShoot, 
      PieceConstants.rightSpeakerPower * PieceConstants.signRightShoot, 
      PieceConstants.leftUpSpeakerFeedPower * PieceConstants.signLeftUpFeed, 
      PieceConstants.rightDownSpeakerFeedPower * PieceConstants.signRightDownFeed));
		/*speaker.whileTrue(new ShootCommand(shootSubsystem, PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower));
    //shoot.onTrue(new IntakeCommand(intakeSubsystem, -0.1, -0.1).withTimeout(0.3));
    speaker.whileTrue(new WaitCommand(1.2). andThen(new IntakeCommand(
      intakeSubsystem, -PieceConstants.leftUpSpeakerFeedPower, -PieceConstants.rightDownSpeakerFeedPower))); */

    // if L2 on piece, shoot for the amp
    Trigger amp = new Trigger(() -> pieceController.getL2Button()); 
    amp.whileTrue(new AmpCommand(intakeSubsystem, shootSubsystem, 
      PieceConstants.leftAmpPower * PieceConstants.signLeftShoot, 
      PieceConstants.rightAmpPower * PieceConstants.signRightShoot, 
      PieceConstants.leftUpAmpFeedPower * PieceConstants.signLeftUpFeed, 
      PieceConstants.rightDownAmpFeedPower * PieceConstants.signRightDownFeed));
    /*amp.whileTrue(new ShootCommand(shootSubsystem, PieceConstants.leftAmpPower, PieceConstants.rightAmpPower));
    amp.whileTrue(new WaitCommand(0.7). andThen(new ManualIntakeCommand(
      intakeSubsystem, PieceConstants.leftUpAmpFeedPower, PieceConstants.rightDownAmpFeedPower)));*/

    // if R2 on drive, pass a game piece
    Trigger pass = new Trigger(() -> pieceController.getR2Button());
    pass.whileTrue(new PassCommand(intakeSubsystem, shootSubsystem, 
      PieceConstants.leftPassPower * PieceConstants.signLeftShoot,
      PieceConstants.rightPassPower * PieceConstants.signRightShoot,
      PieceConstants.leftPassFeedPower * PieceConstants.signLeftUpFeed,
      PieceConstants.rightPassFeedPower * PieceConstants.signRightDownFeed));

    // if L1 on piece, automatically intake a piece
    Trigger intake = new Trigger(() -> pieceController.getL1Button()); 
    intake.onTrue(new ParallelCommandGroup(
      new ManualIntakeCommand(intakeSubsystem, PieceConstants.leftUpIntakePower * PieceConstants.signLeftUpIntake, 
        PieceConstants.rightDownIntakePower * PieceConstants.signRightDownIntake), 
      new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
        PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD)));
    intake.onFalse(new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD));
    intake.whileTrue(new ManualIntakeCommand(intakeSubsystem, -PieceConstants.leftUpIntakePower, PieceConstants.rightDownIntakePower));
    intake.onTrue(new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
      PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD));
    intake.onFalse(new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD));

    // if D-Pad up on piece, auto move intake out; if D-Pad down on piece, auto move intake in
    Trigger autoIntakeOut = new Trigger (() -> pieceController.getPOV() == 0); 
    Trigger autoIntakeIn = new Trigger (() -> pieceController.getPOV() == 180); 
    autoIntakeOut.onTrue(new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle, 
      PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD));
    autoIntakeIn.onTrue(new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
      PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD));

    // manual intake system
    Trigger manualIntake = new Trigger(() -> pieceController.getTriangleButton()); // if Triangle on piece, turn on intake
    Trigger manualIntakeOut = new Trigger(() -> pieceController.getSquareButton()); // if Square on piece, manually move intake out
    Trigger manualIntakeIn = new Trigger(() -> pieceController.getCrossButton()); // if Cross on piece, manually move intake in
    manualIntakeOut.whileTrue(new ManualIntakeMoveCommand(intakeMoveSubsystem, -PieceConstants.intakeMovePower));
    manualIntakeIn.whileTrue(new ManualIntakeMoveCommand(intakeMoveSubsystem, PieceConstants.intakeMovePower));
    manualIntake.whileTrue(new ManualIntakeCommand(intakeSubsystem, 
      PieceConstants.leftUpIntakePower * PieceConstants.signLeftUpIntake, 
      PieceConstants.rightDownIntakePower * PieceConstants.signRightDownIntake));

    // manually move climb mechanism up and down
    /*climbUp.whileTrue(new ClimbCommand(climbSubsystem, PieceConstants.climbPower));
    climbDown.whileTrue(new ClimbCommand(climbSubsystem, -PieceConstants.climbPower));
    fullClimb.whileTrue(new ClimbCommand(climbSubsystem, -PieceConstants.fullClimbPower));*/

    // if Options on drive, reset heading; if Options on piece, reset intake move encoder value; if Share on drive, reverse reset 
    Trigger resetHeading = new Trigger(() -> driveController.getOptionsButton()); 
    Trigger resetIntake = new Trigger(() -> pieceController.getOptionsButton()); 
    Trigger reverseResetHeading = new Trigger(() -> driveController.getShareButton());
    resetHeading.onTrue(new ResetHeadingCommand(drivebase));
    resetIntake.onTrue(new ResetIntakeCommand(intakeMoveSubsystem));
    reverseResetHeading.onTrue(new ReverseResetHeadingCommand(drivebase));

    // if D-Pad left on piece, fully automatic intake system
    /*Trigger fullAutoIntake = new Trigger(() -> pieceController.getPOV() == 270); 
    fullAutoIntake.onTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new IntakeCommand(intakeSubsystem, PieceConstants.leftUpIntakePower * PieceConstants.signLeftUpIntake, 
          PieceConstants.rightDownIntakePower * PieceConstants.signRightDownIntake).withTimeout(3),
        new IntakeOutCommand(intakeMoveSubsystem, PieceConstants.intakeOutAngle,
            PieceConstants.intakeOutP, PieceConstants.intakeOutI, PieceConstants.intakeOutD)), 
      new IntakeInCommand(intakeMoveSubsystem, PieceConstants.intakeInAngle, 
        PieceConstants.IntakeInP, PieceConstants.intakeInI, PieceConstants.intakeInD)));*/

    // Limelight modes
    /*aprilTag.toggleOnTrue(new setAprilTagCommand(limelightSubsystem));
    neuralNetwork.toggleOnTrue(new setNeuralNetworkCommand(limelightSubsystem));
    visionMode.toggleOnTrue(new setVisionModeCommand(limelightSubsystem, LimelightConstants.visionProcessModeId));
    driveMode.toggleOnTrue(new setVisionModeCommand(limelightSubsystem, LimelightConstants.cameraModeId));*/

    // half drive/turn speeds
    Trigger halfSpeed = new Trigger(() -> driveController.getL1Button()); // if L1 on drive, half speed
		Trigger halfTurn = new Trigger (() -> pieceController.getR1Button()); // if R1 on drive, half turn speed	
    Trigger halfMovement = new Trigger(halfSpeed).and(halfTurn); // if L1 and R1 on drive, half movement

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

    // if PS4 Button on piece or drive, stop everything other than movement
    Trigger stopAllArm = new Trigger(() -> driveController.getPSButton()).or(() -> pieceController.getPSButton());
    stopAllArm.whileTrue(new ParallelCommandGroup(
      new ShootCommand(shootSubsystem, 0, 0), 
      new ManualIntakeCommand(intakeSubsystem, 0, 0), 
      new IntakeOutCommand(intakeMoveSubsystem, 0, 0, 0, 0)));
    stopAllArm.onTrue(new ParallelCommandGroup(
      new ShootCommand(shootSubsystem, 0, 0), 
      new ManualIntakeCommand(intakeSubsystem, 0, 0), 
      new IntakeOutCommand(intakeMoveSubsystem, 0, 0, 0, 0)));
          


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

  /*
   * This returns which autonomous routine we chose. 
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /*
   * This sets our motors to brake mode. It is used in Robot.java. 
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
