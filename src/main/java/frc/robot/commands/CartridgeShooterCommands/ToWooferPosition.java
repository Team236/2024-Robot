// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CartridgeShooter;

public class ToWooferPosition extends Command {

   private CartridgeShooter cartridgeShooter;

  /** Creates a new ToWooferShotPosition. */

  public ToWooferPosition(CartridgeShooter cartridgeShooter) {
    this.cartridgeShooter = cartridgeShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridgeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cartridgeShooter.wooferShotPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;  //*** Try false here if having trouble executing this command
  }
}
