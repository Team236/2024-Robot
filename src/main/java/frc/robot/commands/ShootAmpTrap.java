// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.AmpTrapShooter;

public class ShootAmpTrap extends Command {

private double speed;
private Counter counter;
private Robot robot;

  private AmpTrapShooter ampTrapShooter;
  /** Creates a new ShootAmpTrap. */
  public ShootAmpTrap(AmpTrapShooter ampTrapShooter, Robot robot, double speed) {
    this.ampTrapShooter = ampTrapShooter;
    this.robot = robot;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ampTrapShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampTrapShooter.shoot(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampTrapShooter.stop();
    robot.counter.reset();  //TODO - or get from intake if this doesn't work
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
