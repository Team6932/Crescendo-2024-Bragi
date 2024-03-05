package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightDrive extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private final SwerveSubsystem drivebase;

    public LimelightDrive(LimelightSubsystem limelightSubsystem, SwerveSubsystem drivebase) {
        this.limelightSubsystem = limelightSubsystem;
        this.drivebase = drivebase;
        addRequirements(limelightSubsystem, drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drivebase.driveToPose(limelightSubsystem.getLimelightPosition());
    }

    @Override
    public void end (boolean interrupted) {
        drivebase.driveToPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
