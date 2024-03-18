package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PieceConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SpeakerCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final ShootSubsystem shootSubsystem;

    private double leftShoot, rightShoot;
    private double leftFeed, rightFeed;

    public SpeakerCommand(IntakeSubsystem intakeSubsystem, ShootSubsystem shootSubsystem, 
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
    public void initialize() {}

    @Override
    public void execute() {
        shootSubsystem.shoot(leftShoot, rightShoot);
        if (shootSubsystem.getShootReady(PieceConstants.speakerMotorSpeed)) {
            intakeSubsystem.intake(leftFeed, rightFeed);
        } else {
            shootSubsystem.shoot(leftShoot, rightShoot);
        }
    }

    @Override
    public void end (boolean interrupted) {
        shootSubsystem.shoot(0, 0);
        intakeSubsystem.intake(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
