/*
 * This controls the two "rollers"/belt systems on our intake system. 
 * They are used to get pieces into our intake system and feed pieces into our shooting system. 
 */
package frc.robot.commands.IntakeSystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    /*
     * We need the IntakeSubsystem and two doubles controlling the speed of the motors.
     * 
     * The intakeSwitch was a bad idea given our time constraints and other issues. 
     * The limit switch was placed in the back of our intake system.
     * It was intended to prevent the intake motors from running after we got a piece.
     * It would also remove the need for a time-based end condition for our intake during autonomous. 
     */
    private final IntakeSubsystem intakeSubsystem;
    private double leftSpeed;
    private double rightSpeed;
    private boolean intakeSwitch;

    /* 
     * All of the parameters for this command are necessary because this command is used for different functions.
     * Different functions require different speeds. 
    */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double leftSpeed, double rightSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        addRequirements(intakeSubsystem);
    }

    /*
     * Nothing needs to happen when the command begins.
     */
    @Override
    public void initialize() {}

    /*
     * Get if the intake switch is pressed.
     * Get the intake motors to spin at the input speeds. 
     */
    @Override
    public void execute() {
        intakeSwitch = intakeSubsystem.getIntakeSwitch();
        intakeSubsystem.intake(leftSpeed, rightSpeed);
    }

    /*
     * Make the motors stop running.
     */
    @Override
    public void end (boolean interrupted) {
        intakeSubsystem.intake(0.0, 0.0);
    }

    /*
     * When we are trying to intake pieces, the intake motors run in a positive direction. 
     * If we are trying to intake a piece and the limit switch is triggered, end the command. 
     * 
     * If we are tring to shoot a piece, the speeds are negative, so this command would never end.
     * We also never used this command for shooting. 
     */
    @Override
    public boolean isFinished() {
        if (rightSpeed > 0 && leftSpeed > 0 && intakeSwitch) {
            return true;
        } else {
            return false;
        }
    } 
}
