package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private double leftSpeed;
    private double rightSpeed;
    private boolean limitSwitch;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double leftSpeed, double rightSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
 ////////////////////////// CHECK IF LIMIT SWITCH WORKS ///////////////////////////
        /*limitSwitch = intakeSubsystem.getLimitSwitch();
        if (leftSpeed > 0 || rightSpeed > 0) {
            if (limitSwitch != true) {
                intakeSubsystem.intake(0, 0);
            } else {
                intakeSubsystem.intake(leftSpeed, rightSpeed);
            }
        } else {
            intakeSubsystem.intake(leftSpeed, rightSpeed);
        } */

        /* intakeLimit = intakeSubsystem.intakeLimit(); 
        
        if (leftSpeed > 0 || rightSpeed > 0) {
            if (intakeLimit) {
                intakeSubsystem.intake(0, 0);
            } else {
                intakeSubsystem.intake(leftSpeed, rightSpeed);
            }
        } else {
            intakeSubsystem.intake(leftSpeed, rightSpeed);
        } */

        intakeSubsystem.intake(leftSpeed, rightSpeed);
    }

    @Override
    public void end (boolean interrupted) {
        intakeSubsystem.intake(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}
