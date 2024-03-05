package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{
    
    private final ClimbSubsystem climbSubsystem;
    private double speed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double speed) {
        this.climbSubsystem = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climbSubsystem.climb(speed);
    }

    @Override
    public void end (boolean interrupted) {
        climbSubsystem.climb(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}
