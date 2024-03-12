package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class IntakeMoveCommand extends Command{ 
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    
    private double angle;
    private double P;
    private boolean outLimitSwitch, inLimitSwitch;

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
        outLimitSwitch = intakeMoveSubsystem.outLimitSwitch();
        inLimitSwitch = intakeMoveSubsystem.inLimitSwitch();

        if (angle > 120) {
            if (outLimitSwitch) {
                intakeMoveSubsystem.intakeMove(angle, 0);
            } else {
                intakeMoveSubsystem.intakeMove(angle, P);
            }
        } else if (angle < 20) {
            if (inLimitSwitch) {
                intakeMoveSubsystem.intakeMove(angle, 0);
            } else {
                intakeMoveSubsystem.intakeMove(angle, P);
            }
        } else {
            intakeMoveSubsystem.intakeMove(angle, P);
        }
        
        // intakeMoveSubsystem.intakeMove(angle, P);
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
