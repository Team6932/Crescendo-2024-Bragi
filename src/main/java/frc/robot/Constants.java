// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  public static final double kMaxDriveSpeed = 4.8; // m/s
  public static final double kMaxTurnSpeed = 2 * Math.PI; // rad/s

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    public static final double drivePowerPercent = 0.85;
    public static final double turnPowerPercent = 0.7;
  }


  public static class PieceConstants {

    // CAN IDs
    public static final int rightShootId = 12;
    public static final int leftShootId = 13;
    public static final int intakeMoveId = 9;
    public static final int intakeRightDownId = 10;
    public static final int intakeLeftUpId = 11;
    public static final int climb = 14;

    // power settings (-1 to 1)
    public static final double leftSpeakerPower = 0.8; // orange wheels on the left for speaker 
    public static final double rightSpeakerPower = 0.8; // orange wheels on the right for speaker

    public static final double leftAmpPower = 0.4; // orange wheels on left for amp
    public static final double rightAmpPower = 0.4; // orange wheels on right for amp

    public static final double leftUpIntakePower = 0.3; // intake pieces on the left/top
    public static final double rightDownIntakePower = 0.3; // intake pieces on the right/bottom

    public static final double leftUpSpeakerFeedPower = 0.85; // move pieces from intake to shooter for speaker on left/top
    public static final double rightDownSpeakerFeedPower = 0.85; // move pieces from intake to shooter for speaker on right/bottom

    public static final double leftUpAmpFeedPower = 0.4; // move piece from intake to shooter for amp on left/top
    public static final double rightDownAmpFeedPower = 0.4; // move pieces from intake to shooter for amp on right/bottom
    
    public static final double intakeMovePower = 0.5; // move entire intake system 
    public static final double climbPower = 0.3; // move entire climb mechanism
    public static final double fullClimbPower = 0.75;

    // other game piece related constants
    public static final double intakeOutAngle = 152.0;
    public static final double intakeInAngle = 0.0;
  }

  public static class LimelightConstants {

    // Limelight position constants
    public static final double limelightAngle = 0.0; // limelight is level
    public static final double limelightHeight = 0.23; // height of limelight from ground in meters
  }
}
