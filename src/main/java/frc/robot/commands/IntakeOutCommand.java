package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends Command{ 
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    
    private double angle;
    private double P, I, D;
    private boolean limitSwitch;

    public IntakeOutCommand(IntakeMoveSubsystem intakeMoveSubsystem, IntakeSubsystem intakeSubsystem, double angle, double P, double I, double D) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
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

        /*limitSwitch = intakeSubsystem.getLimitSwitch();
        
        if (limitSwitch != true) {
            intakeMoveSubsystem.intakeMove(0, P, I, D);
        } else {
            intakeMoveSubsystem.intakeMove(angle, P, I, D);
        } */

        // outLimitSwitch = intakeMoveSubsystem.outLimitSwitch();
        // inLimitSwitch = intakeMoveSubsystem.inLimitSwitch();
        /*limitSwitch = intakeSubsystem.intakeLimit();

        if (limitSwitch) {
            intakeMoveSubsystem.intakeMove(0, 0.05);
        } else {
            intakeMoveSubsystem.intakeMove(angle, P);
        } */
        
        /*
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
        } */
        
        intakeMoveSubsystem.intakeMove(angle, P, I, D); // THIS LINE IS THE OLD CODE
    }

    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.simpleIntakeMove(0);
        //intakeMoveSubsystem.intakeMove(angle, 0.0, 0.0, 0.0);
    }

    @Override 
    public boolean isFinished () {
        if (intakeMoveSubsystem.getIntakeEncoder() <= angle) {
            return true;
        } else {
            return false;
        } 
        //return false;
    } 
}
