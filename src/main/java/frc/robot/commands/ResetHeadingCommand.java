package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetHeadingCommand extends Command {
    private final SwerveSubsystem drivebase;

    public ResetHeadingCommand (SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

    @Override
    public void initialize () {}

    @Override 
    public void execute () {
        drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
