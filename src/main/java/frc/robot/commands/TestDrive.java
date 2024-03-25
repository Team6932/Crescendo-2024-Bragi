package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TestDrive extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private final SwerveSubsystem drivebase;

    public TestDrive(LimelightSubsystem limelightSubsystem, SwerveSubsystem drivebase) {
        this.limelightSubsystem = limelightSubsystem;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        limelightSubsystem.getAlliance();
        drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    @Override
    public void execute() {
        drivebase.doubleDriveCommand(
            limelightSubsystem.getXMeters() / LimelightConstants.maxXDist, 
            limelightSubsystem.getYMeters() / LimelightConstants.maxYDist, 
            limelightSubsystem.getRot());
    }

    @Override
    public void end (boolean interrupted) {
        drivebase.doubleDriveCommand(0, 0, 0);
        drivebase.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)));
    }

    @Override
    public boolean isFinished () {
        if (limelightSubsystem.getInPosition()) {
            return true;
        } else {
            return false;
        }
    }
}
