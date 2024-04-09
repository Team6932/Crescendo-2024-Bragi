// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * This file contains all the constants we used/wanted to use. 
 * 
 * double is a number that can have decimals
 * int is an integer (no decimals)
 * 
 * public means everything can be accessed by other classes (files)
 * learn more about Java for static and final 
 * 
 * An option for better orginization is to have a Constants folder that contains 
 * different files. Each file can have constants relating to a specific task or mechanism.
 */

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /*
   * These were from YAGSL
   * I only changed the ones we needed
   * PID values are not tuned
   */
  public static final double kMaxDriveSpeed = 4.6; // m/s
  public static final double kMaxTurnSpeed = 2 * Math.PI; // rad/s

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.5, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    /*
     * The deadband affects how much the driver must move the joystick before the robot reacts
     * 
     * drivePowerPercent and turnPowerPercent do not actually slow the robot by a specific percent.
     * They limit the joystick output to the drive function, resulting in a decrease in speed.
     */
    // deadband
    public static final double LEFT_X_DEADBAND  = 0.15;
    public static final double LEFT_Y_DEADBAND  = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double RIGHT_Y_DEADBAND = 0.15;
    public static final double TURN_CONSTANT    = 6;

    public static final double drivePowerPercent = 1.00;
    public static final double turnPowerPercent = 0.85;
  }


  public static class PieceConstants {

    /*
     * These say if the motor needs to spin in a positive or negative direction to perform a given task.
     * I created this section since we were changing how we were mounting our motors.
     * I did not want to risk accidentally missing or adding a negative at a competition.
     *  
     * Another strategy would be to have these be booleans (true/false)
     * and do SparkMAX.setInverted(true/false);
     */
    // motor positive/negative settings
    public static final double signRightShoot = 1;
    public static final double signLeftShoot = 1;

    public static final double signLeftUpIntake = 1;
    public static final double signRightDownIntake = 1;

    public static final double signLeftUpFeed = -1;
    public static final double signRightDownFeed = -1;

    /*
     * These set the smart current limit for the SparkMAX motor controllers.
     * The motors were getting too hot due to the lack of a current limit on the SparkMAX. 
     */
    // smart current limit
    public static final int intakeCurrent = 60;
    public static final int intakeMoveCurrent = 60;
    public static final int shootCurrent = 60;

    /*
     * These are the CAN IDs for all of our intake/shoot related motor controllers
     */
    // CAN IDs
    public static final int rightShootId = 12;
    public static final int leftShootId = 13;
    public static final int intakeMoveId = 9; 
    public static final int intakeRightDownId = 11; 
    public static final int intakeLeftUpId = 14; 
    public static final int climb = 10; 

    /*
     * These are the DIO (digital input) IDs for our limit switches. We ended up only partially using one limit switch.
     * There should also be a way to connect limit switches to the motor controllers. 
     */
    // limit switch IDs
    public static final int intakeSwitch = 1;
    public static final int intakeOutSwitch = 7;
    public static final int intakeInSwitch = 8;

    /*
     * This contains all the power settings for our different mechanisms. These are are done through testing.
     */
    // power settings (-1 to 1)
    public static final double leftSpeakerPower = 0.8; // orange wheels on the left for speaker 
    public static final double rightSpeakerPower = 0.8; // orange wheels on the right for speaker
    public static final double leftUpSpeakerFeedPower = 0.85; // move pieces from intake to shooter for speaker on left/top
    public static final double rightDownSpeakerFeedPower = 0.85; // move pieces from intake to shooter for speaker on right/bottom

    public static final double leftAmpPower = 0.13; // orange wheels on left for amp
    public static final double rightAmpPower = 0.10; // orange wheels on right for amp
    public static final double leftUpAmpFeedPower = 0.17; // move piece from intake to shooter for amp on left/top
    public static final double rightDownAmpFeedPower = 0.17; // move pieces from intake to shooter for amp on right/bottom

    public static final double leftPassPower = 1; // orange wheels on left for passing
    public static final double rightPassPower = 1; // orange wheels on right for passing
    public static final double leftPassFeedPower = 1; // move piece from intake to shooter for passing on left
    public static final double rightPassFeedPower = 1; // move piece from intake to shooter for passing on right

    public static final double leftUpIntakePower = 0.5; // intake pieces on the left/top 
    public static final double rightDownIntakePower = 0.5; // intake pieces on the right/bottom
    
    public static final double intakeMovePower = 0.35; // move entire intake system manually

    public static final double climbPower = 0.3; // UNUSED move entire climb mechanism
    public static final double fullClimbPower = 0.75; // UNUSED

    /*
     * These contain desired encoder velocity values obtained from SparkMAX.getEncoder().getVelocity(). 
     * These values were obtained through testing. 
     * You can use setVelocityConversionFactor() to convert them into more useful units like m/s or ft/s. 
     * 
     * Originally, the plan was to use PID and encoder velocities to control the motors.
     * However, I didn't want to deal with PID constants.
     * Encoder velocities were used to check if the motors reached the desired speed. 
     */
    public static final double speakerMotorSpeed = 3800; // rpm of the motors to make it to the speaker, wheels 4 in diameter, 4400
    public static final double leftAmpMotorSpeed = 670; // rpm of left motor to make it to the amp
    public static final double rightAmpMotorSpeed = 480; // rpm of right motor for amp
    public static final double passMotorSpeed = 5200; // rpm of the motors to pass
    public static final double maxSpeakerMotorSpeed = 5500; // hypothetical max rpm of the motors

    /*
     * These are PID constants for moving the intake in/out.
     * These are not tuned (DO NOT COPY THESE). It worked, so I never bothered to tune them properly. 
     */
    // PID settings
    public static final double intakeOutP = 0.5;
    public static final double intakeOutI = 0.5;
    public static final double intakeOutD = 0.0;

    public static final double IntakeInP = 0.5;
    public static final double intakeInI = 0.5;
    public static final double intakeInD = 0.0;

    /*
     * This limits what the SparkMAX tries to tell the motor to do.
     * In this case, the power setting will always be between -0.80 and 0.80 (look in IntakeMoveSubsystem to see how).
     */
    public static final double maxIntakeMovePID = 0.80; 

    /*
     * These are the setpoints used for moving the intake in/out.
     * These are values obtained from SparkMAX.getEncoder().getPosition().
     * 
     * You can turn these values into more useful units like degrees or radians using setPositionConversionFactor(). 
     */
    // other game piece related constants
    public static final double intakeOutAngle = -30.0; // encoder position value when intake is out
    public static final double intakeInAngle = 0.0; // angle when intake is retracted
  }

  public static class LimelightConstants {

    // Limelight IDs and modes
    public static final int cameraModeId = 1;
    public static final int visionProcessModeId = 0;

    // Limelight ranges
    public static final double xFOV = 82; // degrees for left and right
    public static final double yFOV = 56; // degrees for up and down

    // Limelight position constants
    public static final double limelightAngle = 0.0; // limelight is level
    public static final double limelightHeight = 0.23; // height of limelight from ground in meters

    // field element constants
    public static final double subwoofer = 0.92; // distance of the thing that sticks out from the speaker in m
    public static final double tagHeight = 1.22; // height of apriltags in m 

    public static final int blueSpeakerId = 7; // april tag id of mid blue speaker
    public static final int redSpeakerId = 4; // april tag id of mid red speaker

    public static final double maxXDist = 1;
    public static final double maxYDist = 1;
    public static final double maxRotDeg = 27;

    // acceptable amount of error in m and deg
    public static final double xDistError = 0.2;
    public static final double yDistError = 0.2;
    public static final double rotDegError = 3;

  }
}
