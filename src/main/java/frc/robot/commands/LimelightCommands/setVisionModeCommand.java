package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class setVisionModeCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;
    private final int camModeId;

    public setVisionModeCommand (LimelightSubsystem limelightSubsystem, int camModeId) {
        this.limelightSubsystem = limelightSubsystem;
        this.camModeId = camModeId;
    }

    @Override
    public void initialize () {}
    
    @Override
    public void execute () {
        limelightSubsystem.setVisionMode(camModeId);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished () {
        return false;
    }
}
