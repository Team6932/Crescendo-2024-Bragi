/*
 * This manually moves the intake system in/out without any PID. 
 */
package frc.robot.commands.IntakeSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class ManualIntakeMoveCommand extends Command{ 
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    private double speed;

    /*
     * We need the subsystem and a speed to set the motor to.
     */
    public ManualIntakeMoveCommand(IntakeMoveSubsystem intakeMoveSubsystem, double speed) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.speed = speed;
        addRequirements(intakeMoveSubsystem);
    }

    /*
     * Nothing needs to happen when the command begins.
     */
    @Override
    public void initialize() {}

    /*
     * Simply set the motor controlling intake in/out to the inputed speed. 
     */
    @Override
    public void execute () {
        intakeMoveSubsystem.simpleIntakeMove(speed);
    }

    /*
     * Make the motor stop moving.
     */
    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.simpleIntakeMove(0.0);
    }

    /*
     * This command never naturally ends. 
     */
    @Override
    public boolean isFinished() {
        return false; 
    } 
}
