/*
 * This is a command that sets the robot's heading to 0 (forward). The current direction the robot is facing becomes forward.
 * Even though this is a very simple command, I wanted a separate file for organization purposes. 
 */

package frc.robot.commands.DriveSystemCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetHeadingCommand extends Command {
    /*
     * The only thing this command needs is the the SwerveSubsystem.
     * "swerveSubsystem" would have been a better name than "drivebase" for consistent naming practices. 
     */
    private final SwerveSubsystem drivebase;

    public ResetHeadingCommand (SwerveSubsystem drivebase) {
        /*
         * This command does not 
         */
        this.drivebase = drivebase;
    }

    /*
     * Nothing needs to happen when the command begins.
     */
    @Override
    public void initialize () {}

    /*
     * This completely resets the robot pose to 0, 0, 0. 
     */
    @Override 
    public void execute () {
        drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    }

    /*
     * Nothing needs to happen when the command ends.
     */
    @Override 
    public void end (boolean interrupted) {}

    /*
     * Make isFinished always return true, so the command only runs once.
     * Creating an "InstantCommand" instead of a standard "Command" would also work. 
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
