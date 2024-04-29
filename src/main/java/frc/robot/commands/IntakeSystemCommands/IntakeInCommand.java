/*
 * This automatically moves our intake system out using PID. 
 * 
 * This command is not dummy proof if you try to run it without properly resetting the encoder value.
 * The encoder value should be 0 when the intake is retracted. 
 * 
 * Mechanical switches are the best way to make this safer. The encoder values could also be set to specific values 
 * when switches are triggered. This way, the human drivers do not have to worry about resetting values. 
 */
package frc.robot.commands.IntakeSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class IntakeInCommand extends Command {
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;

    private double angle;
    private double P, I, D;
    //private boolean intakeInSwitch;

    /* 
     * The input parameters include the subsystem and the PID constants. 
     * The parameters have the same flaw as SpeakerCommand.java (I don't know why I really liked having inefficient inputs). 
     * Since this is specifically for moving the intake system out, we can directly call the PID settings from Constants.java. 
     */
    public IntakeInCommand(IntakeMoveSubsystem intakeMoveSubsystem, double angle, double P, double I, double D) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.angle = angle;
        this.P = P;
        this.I = I;
        this.D = D;
        addRequirements(intakeMoveSubsystem);
    }

    /*
     * Nothing needs to happen when the command begins.
     */
    @Override
    public void initialize() {}

    /*
     * Use PID to move the intake system out. 
     */
    @Override
    public void execute() {
        //intakeInSwitch = intakeMoveSubsystem.getIntakeInSwitch();
        intakeMoveSubsystem.intakeMove(angle, P, I, D);
    }

    /*
     * Make the intake system stop moving in/out. 
     * I used the simple motor.set(0) to make sure no more errors happened with PID going haywire. 
     */
    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.simpleIntakeMove(0);
    }

    /*
     * If the position encoder associated with moving the intake system in/out has reached the setpoint, end the command. 
     */
    @Override
    public boolean isFinished() {
        if (intakeMoveSubsystem.getIntakeEncoder() >= angle){ //|| intakeInSwitch) { 
            return true;
        } else {
            return false;
        }
    }
}
