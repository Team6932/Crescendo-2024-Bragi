// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class Shoot extends Command {

    private final ShootSubsystem shootSubsystem;
    private double leftShoot;
    private double rightShoot;

  public Shoot(ShootSubsystem shootSubsystem, double leftShoot, double rightShoot) {
    this.shootSubsystem = shootSubsystem;
    this.leftShoot = leftShoot;
    this.rightShoot = rightShoot;
    addRequirements(shootSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootSubsystem.shoot(leftShoot, rightShoot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootSubsystem.shoot(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
} 
