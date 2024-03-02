package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class IntakeMoveCommand extends Command{ 
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    
    private double angle;
    private double P;

    public IntakeMoveCommand(IntakeMoveSubsystem intakeMoveSubsystem, double angle, double P) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.angle = angle;
        this.P = P;
        addRequirements(intakeMoveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute () {
        intakeMoveSubsystem.intakeMove(angle, P);
    }

    @Override
    public void end (boolean interrupted) {
        // intakeMoveSubsystem.intakeMove(0.0, 0.0);
    }

    @Override 
    public boolean isFinished () {
        return false;
    } 
}
