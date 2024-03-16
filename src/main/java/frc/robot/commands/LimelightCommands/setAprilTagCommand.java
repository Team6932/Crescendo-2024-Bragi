package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class setAprilTagCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;

    public setAprilTagCommand (LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
    }

    @Override
    public void initialize () {}
    
    @Override
    public void execute () {
        limelightSubsystem.setAprilTag();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished () {
        return false;
    }
}
