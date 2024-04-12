/*
 * This does the same thing as ResetHeadingCommand.java but reverse.
 * Refer to ResetHeadingCommand.java for comments about how it works. 
 * 
 * The robot's current heading becomes the backwards direction.
 * We never used this (plus I didn't want more buttons for the drivers). I just felt like adding it. 
 */
package frc.robot.commands.DriveSystemCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ReverseResetHeadingCommand extends Command {
    private final SwerveSubsystem drivebase;

    public ReverseResetHeadingCommand (SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

    @Override
    public void initialize () {}

    @Override 
    public void execute () {
        drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)));
    }

    @Override 
    public void end (boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
