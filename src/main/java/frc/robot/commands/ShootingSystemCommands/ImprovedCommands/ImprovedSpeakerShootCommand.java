/*
 * This is one better way of programming a command designed to shoot for the speaker. 
 * Instead of having motor powers as an input parameter, they are directly set in this file. 
 * This can greatly reduce the amount of inefficient text in RobotContainer.java. 
 * 
 * The functionality of the command is not changed. 
 */

package frc.robot.commands.ShootingSystemCommands.ImprovedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PieceConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ImprovedSpeakerShootCommand extends Command{
    /*
     * This commmand uses two subsystems, a lot of doubles, a boolean, and a timer. 
     */
    private final IntakeSubsystem intakeSubsystem;
    private final ShootSubsystem shootSubsystem;

    private double leftShoot = PieceConstants.signLeftShoot * PieceConstants.leftSpeakerPower;
    private double rightShoot = PieceConstants.signRightShoot * PieceConstants.rightSpeakerPower;

    private double leftFeed = PieceConstants.signLeftUpFeed * PieceConstants.leftUpSpeakerFeedPower;
    private double rightFeed = PieceConstants.signRightDownFeed * PieceConstants.rightDownSpeakerFeedPower;

    private boolean spaghettiIfStatement;
    private Timer time = new Timer();

    /*
     * The inputs for this command only include the required subsystems.
     */
    public ImprovedSpeakerShootCommand(IntakeSubsystem intakeSubsystem, ShootSubsystem shootSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shootSubsystem = shootSubsystem;
        addRequirements(intakeSubsystem, shootSubsystem);
    }

    /*
     * This resets the timer and sets the boolean to be true.
     */
    @Override
    public void initialize() {
        spaghettiIfStatement = true;
        time.reset();
    }

    /*
     * While the command is running, the shooter wheels should always be running. 
     * 
     * After the shooter wheels reach the desired speed, the timer is reset and the boolean is set to false.
     * 
     * When the boolean is false, the motors on the intake will start running and feed the note into the shooter wheels.
     * 
     * If the shooter wheels have been running for a while and have not reached the desired speed, the intake will start running.
     */
    @Override
    public void execute() {
        shootSubsystem.shoot(leftShoot, rightShoot);

        if (shootSubsystem.getShootReady(PieceConstants.speakerMotorSpeed) && spaghettiIfStatement) {
            time.reset();
            spaghettiIfStatement = false;

        } else if (!spaghettiIfStatement) {
            intakeSubsystem.intake(leftFeed, rightFeed);

        } else if (time.hasElapsed(1.5)) {
            intakeSubsystem.intake(leftFeed, rightShoot);
            time.reset();
            spaghettiIfStatement = false;

        } else {
            intakeSubsystem.intake(0, 0);
        }
    }

    /*
     * Set all the motors to 0 after the command ends
     */
    @Override
    public void end (boolean interrupted) {
        shootSubsystem.shoot(0, 0);
        intakeSubsystem.intake(0, 0);
    }

    /*
     * This command ends when the boolean is false and the timer has reached at least 0.2 seconds. 
     * 
     * The boolean is only set to false after the shooter wheels have reached the desired velocity or after a long time. 
     * The command will end after the intake motors have tried to feed the note into the shooter for 0.2 seconds. 
     */
    @Override
    public boolean isFinished() {
        if (time.hasElapsed(0.2) && !spaghettiIfStatement) {
            return true;
        } else {
            return false;
        } 
    }
}
