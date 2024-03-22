package frc.robot.commands.IntakeSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class ResetIntakeCommand extends Command {
    private final IntakeMoveSubsystem intakeMoveSubsystem;

    public ResetIntakeCommand(IntakeMoveSubsystem intakeMoveSubsystem) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeMoveSubsystem.resetEncoder();
    }
    
    @Override
    public void end (boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
