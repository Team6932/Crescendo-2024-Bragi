package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootAutoCommand extends Command{
    private ShootSubsystem shootSubsystem;
    private double rpm;
    private double P, I, D;

    public ShootAutoCommand(ShootSubsystem shootSubsystem, double rpm, double P, double I, double D) {
        this.shootSubsystem = shootSubsystem;
        this.rpm = rpm;
        this.P = P;
        this.I = I;
        this.D = D;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shootSubsystem.shootAuto(rpm, P, I, D);
    }

    @Override
    public void end (boolean interrupted) {
        shootSubsystem.shoot(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
