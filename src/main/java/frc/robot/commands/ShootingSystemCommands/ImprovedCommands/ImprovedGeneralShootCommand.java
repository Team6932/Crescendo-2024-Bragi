/*
 * This is another way of programming the robot to shoot.
 * This one command can be used for all of our shooting purposes. 
 * This can greatly reduce the amount of inefficient text in RobotContainer.java. 
 * 
 * There are multiple arrays that contain information about each power setting we want to shoot at. 
 * The power settings are from Constants.java. 
 * Currently, Pass is full power, Speaker is high power, and Amp is low power
 * If you want to add more power settings, simply create more arrays. 
 * 
 * The functionality of the command is not changed. 
 */

package frc.robot.commands.ShootingSystemCommands.ImprovedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PieceConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ImprovedGeneralShootCommand extends Command{
    /*
     * This commmand uses two subsystems, a lot of doubles, three double arrays, two booleans, a string and a timer. 
     */
    private final IntakeSubsystem intakeSubsystem;
    private final ShootSubsystem shootSubsystem;

    private double[] speakerPowers = {PieceConstants.leftSpeakerPower, PieceConstants.rightSpeakerPower, 
        PieceConstants.leftUpSpeakerFeedPower, PieceConstants.rightDownSpeakerFeedPower};
    private double[] ampPowers = {PieceConstants.leftAmpPower, PieceConstants.rightAmpPower,
        PieceConstants.leftUpAmpFeedPower, PieceConstants.rightDownAmpFeedPower};
    private double[] passPowers = {PieceConstants.leftPassPower, PieceConstants.rightPassPower,
        PieceConstants.leftPassFeedPower, PieceConstants.rightPassFeedPower};

    private double leftShoot, rightShoot;
    private double leftFeed, rightFeed;

    private boolean spaghettiIfStatement;
    private Timer time = new Timer();

    private boolean shootModeIssue;

    private String shootMode;

    /*
     * The inputs for this command include the required subsystems and a string.
     */
    public ImprovedGeneralShootCommand(IntakeSubsystem intakeSubsystem, ShootSubsystem shootSubsystem, String shootMode) {
        this.intakeSubsystem = intakeSubsystem;
        this.shootSubsystem = shootSubsystem;
        this.shootMode = shootMode;
        addRequirements(intakeSubsystem, shootSubsystem);
    }

    /*
     * This resets the timer and sets the boolean to be true.
     * Depending on the input string, the power settings used in this command will be set to one of the double arrays. 
     * 
     * If the input string is not valid (not speaker, amp, or pass), set all power settings to 0 and shootModeIssue to true. 
     * 
     * Multiply the power settings by 1 or -1.
     * Since we changed how we mounted our motors relatively frequently, I had to invert/uninvert motors a lot. 
     * Changing a constant from (1 to -1) or (-1 to 1) is a lot easier than adding/removing negatives everywhere. 
     */
    @Override
    public void initialize() {
        spaghettiIfStatement = true;
        time.reset();
        shootModeIssue = false;
        
        if (shootMode == "speaker") {
            leftShoot = speakerPowers[0];
            rightShoot = speakerPowers[1];
            leftFeed = speakerPowers[2];
            rightFeed = speakerPowers[3];
            
        } else if (shootMode == "amp") {
            leftShoot = ampPowers[0];
            rightShoot = ampPowers[1];
            leftFeed = ampPowers[2];
            rightFeed = ampPowers[3];

        } else if (shootMode == "pass") {
            leftShoot = passPowers[0];
            rightShoot = passPowers[1];
            leftFeed = passPowers[2];
            rightFeed = passPowers[3];
        } else {
            leftShoot = 0.0;
            rightShoot = 0.0;
            leftFeed = 0.0;
            rightFeed = 0.0;
            shootModeIssue = true;
        }

        leftShoot *= PieceConstants.signLeftShoot;
        rightShoot *= PieceConstants.signRightShoot;
        leftFeed *= PieceConstants.signLeftUpFeed;
        rightFeed *= PieceConstants.signRightDownFeed;
    }

    /*
     * While the command is running, the shooter wheels should always be running. 
     * 
     * After the shooter wheels reach the desired speed, the timer is reset and the boolean is set to false.
     * 
     * When the boolean is false, the motors on the intake will start running and feed the note into the shooter wheels.
     * 
     * If the shooter wheels have been running for a while and have not reached the desired speed, the intake will start running.
     * 
     * If we did not receive a valid input string (shootModeIssue), display a warning to DriverStation. 
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

        if (shootModeIssue) {
            DriverStation.reportWarning("InvalidShootMode", false);
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
