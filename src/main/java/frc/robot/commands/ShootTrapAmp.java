// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapAmpShooter;

public class ShootTrapAmp extends Command {
  private TrapAmpShooter trapAmpShooter;
  /** Creates a new ShootTrapAmp. */
  public ShootTrapAmp(TrapAmpShooter trapAmpShooter) {
    this.trapAmpShooter = trapAmpShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(trapAmpShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trapAmpShooter.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trapAmpShooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
