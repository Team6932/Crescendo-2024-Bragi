package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private double speed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeSubsystem.intake(speed);
    }

    @Override
    public void end (boolean interrupted) {
        intakeSubsystem.intake(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
