package frc.robot.commands.ShootingSystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PieceConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class PassCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final ShootSubsystem shootSubsystem;

    private double leftShoot, rightShoot;
    private double leftFeed, rightFeed;

    private boolean spaghettiIfStatement;
    private Timer time = new Timer();

    public PassCommand(IntakeSubsystem intakeSubsystem, ShootSubsystem shootSubsystem, 
            double leftShoot, double rightShoot, double leftFeed, double rightFeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.shootSubsystem = shootSubsystem;
        this.leftShoot = leftShoot;
        this.rightShoot = rightShoot;
        this.leftFeed = leftFeed;
        this.rightFeed = rightFeed;
        addRequirements(intakeSubsystem, shootSubsystem);
    }

    @Override
    public void initialize() {
        spaghettiIfStatement = true;
        time.reset();
    }

    @Override
    public void execute() {
        shootSubsystem.shoot(leftShoot, rightShoot);

        if (shootSubsystem.getShootReady(PieceConstants.passMotorSpeed) && spaghettiIfStatement) {
            time.reset();
            spaghettiIfStatement = false;

        } else if (!spaghettiIfStatement) {
            intakeSubsystem.intake(leftFeed, rightFeed);

        } else {
            intakeSubsystem.intake(0, 0);
        }
    }

    @Override
    public void end (boolean interrupted) {
        shootSubsystem.shoot(0, 0);
        intakeSubsystem.intake(0, 0);
    }

    @Override
    public boolean isFinished() {
        if (time.hasElapsed(0.2) && !spaghettiIfStatement) {
            return true;
        } else {
            return false;
        } 
    }
}
