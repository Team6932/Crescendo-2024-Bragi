package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class IntakeInCommand extends Command {
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;

    private double angle;
    private double P, I, D;
    private boolean intakeInSwitch;


    public IntakeInCommand(IntakeMoveSubsystem intakeMoveSubsystem, double angle, double P, double I, double D) {
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
    public void execute() {
        intakeMoveSubsystem.intakeMove(angle, P, I, D);
    }

    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.simpleIntakeMove(0);
    }

    @Override
    public boolean isFinished() {
        if (intakeMoveSubsystem.getIntakeEncoder() + 4 >= angle || intakeInSwitch) { // 4 is offset reading when testing the command
            return true;
        } else {
            return false;
        }
    }
}
