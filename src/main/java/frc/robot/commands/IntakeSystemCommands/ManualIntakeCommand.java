/*
 * This controls the two "rollers"/belt systems on our intake system.
 * It is nearly identical to IntakeCommand.java (no limit switch here). 
 */
package frc.robot.commands.IntakeSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntakeCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private double leftSpeed;
    private double rightSpeed;
    
    /*
     * We need the subsystem and two double to control motor speeds.
     */
    public ManualIntakeCommand(IntakeSubsystem intakeSubsystem, double leftSpeed, double rightSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        addRequirements(intakeSubsystem);
    }

    /*
     * Nothing needs to happen when the command starts.
     */
    @Override
    public void initialize() {}

    /*
     * Set the motors to run at the inputed speeds.
     */
    @Override
    public void execute() {
        intakeSubsystem.intake(leftSpeed, rightSpeed);
    }

    /*
     * Stop all the motors.
     */
    @Override
    public void end (boolean interrupted) {
        intakeSubsystem.intake(0.0, 0.0);
    }

    /*
     * The command never naturally ends. 
     */
    @Override
    public boolean isFinished() {
        return false;
    } 
}
