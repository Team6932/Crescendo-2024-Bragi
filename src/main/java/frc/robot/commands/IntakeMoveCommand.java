// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;

public class IntakeMoveCommand extends Command{
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    private double speed;

    public IntakeMoveCommand(IntakeMoveSubsystem intakeMoveSubsystem, double speed) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.speed = speed;
        addRequirements(intakeMoveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute () {
        intakeMoveSubsystem.intakeMove(speed);
    }

    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.intakeMove(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
