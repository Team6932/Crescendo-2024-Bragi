package frc.robot.commands.IntakeSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class IntakeOutCommand extends Command{ 
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    
    private double angle;
    private double P, I, D;
    private boolean intakeOutSwitch;

    public IntakeOutCommand(IntakeMoveSubsystem intakeMoveSubsystem, double angle, double P, double I, double D) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.angle = angle;
        this.P = P;
        this.I = I;
        this.D = D;
        addRequirements(intakeMoveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute () {
        intakeMoveSubsystem.intakeMove(angle, P, I, D);
    }

    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.simpleIntakeMove(0);
    }

    @Override 
    public boolean isFinished () {
        if (intakeMoveSubsystem.getIntakeEncoder() <= angle || intakeOutSwitch) {
            return true;
        } else {
            return false;
        } 
    } 
}
