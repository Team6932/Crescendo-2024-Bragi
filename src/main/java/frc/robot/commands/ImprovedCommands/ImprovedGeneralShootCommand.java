/*
 * This is another way of programming the robot to shoot.
 */

package frc.robot.commands.ImprovedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PieceConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ImprovedGeneralShootCommand extends Command{
    /*
     * This commmand uses two subsystems, a lot of doubles, three double arrays, a boolean, a string and a timer. 
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

    private String shootMode;

    /*
     * The inputs for this command only include the required subsystems.
     */
    public ImprovedGeneralShootCommand(IntakeSubsystem intakeSubsystem, ShootSubsystem shootSubsystem, String shootMode) {
        this.intakeSubsystem = intakeSubsystem;
        this.shootSubsystem = shootSubsystem;
        this.shootMode = shootMode;
        addRequirements(intakeSubsystem, shootSubsystem);
    }

    /*
     * This resets the timer and sets the boolean to be true.
     */
    @Override
    public void initialize() {
        spaghettiIfStatement = true;
        time.reset();
        
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
            SmartDashboard.putBoolean("InvalidShootMode", true);
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
