// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeMoveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SimpleIntakeMoveCommand extends Command{ 
    
    private final IntakeMoveSubsystem intakeMoveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private boolean limitSwitch;
    private double speed;

    public SimpleIntakeMoveCommand(IntakeMoveSubsystem intakeMoveSubsystem, IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeMoveSubsystem = intakeMoveSubsystem;
        this.speed = speed;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeMoveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute () {

        /*limitSwitch = intakeSubsystem.getLimitSwitch();
        
        if (speed < 0) {
            if (limitSwitch != true) {
                intakeMoveSubsystem.simpleIntakeMove(0);
            }
        } else {
            intakeMoveSubsystem.simpleIntakeMove(speed);
        } */

        /* limitSwitch = intakeSubsystem.intakeLimit();
        
        if (speed > 0) {
            if (limitSwitch) {
                intakeMoveSubsystem.simpleIntakeMove(0);
            }
        } else {
            intakeMoveSubsystem.simpleIntakeMove(speed);
        } */

        // outLimitSwitch = intakeMoveSubsystem.outLimitSwitch();
        // inLimitSwitch = intakeMoveSubsystem.inLimitSwitch();

        /* if (speed > 0) {
            if (outLimitSwitch) {
                intakeMoveSubsystem.simpleIntakeMove(0);
            } else {
                intakeMoveSubsystem.simpleIntakeMove(speed);
            }
        } else {
            if (inLimitSwitch) {
                intakeMoveSubsystem.simpleIntakeMove(0);
            } else {
                intakeMoveSubsystem.simpleIntakeMove(speed);
            }
        } */
        
        intakeMoveSubsystem.simpleIntakeMove(speed); //THIS LINE IS THE OLD CODE
    }

    @Override
    public void end (boolean interrupted) {
        intakeMoveSubsystem.simpleIntakeMove(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; 
    } 
}
