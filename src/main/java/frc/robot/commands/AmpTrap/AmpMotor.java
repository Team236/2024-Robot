// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpTrap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Intake;

public class AmpMotor extends Command {

private double speed;
private AmpTrap ampTrap;

  /** Creates a new AmpMotor. */
  //Spins just the Amp motor, at a set speed
  public AmpMotor(AmpTrap ampTrap, double speed) {
    this.ampTrap = ampTrap;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ampTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampTrap.shoot(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the shooter motor and reset the Note Count to zero, after shooting
    ampTrap.stop();
    Intake.resetCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
