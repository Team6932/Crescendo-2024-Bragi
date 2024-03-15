package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class setCameraCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;

    public setCameraCommand (LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
    }

    @Override
    public void initialize () {}
    
    @Override
    public void execute () {
        limelightSubsystem.setCamera();
    }

    @Override
    public boolean isFinished () {
        return false;
    }
}
