// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpTrap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpTrapShooter;
import frc.robot.subsystems.Intake;

public class ShootAmpTrap extends Command {

private double speed;
private Intake intake;

  private AmpTrapShooter ampTrapShooter;

  /** Creates a new ShootAmpTrap. */
  public ShootAmpTrap(AmpTrapShooter ampTrapShooter, Intake intake, double speed) {
    this.ampTrapShooter = ampTrapShooter;
    this.speed = speed;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ampTrapShooter);
    addRequirements(this.intake);
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
    //stop the shooter motor and reset the Note Count to zero, after shooting
    ampTrapShooter.stop();
    intake.counter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
