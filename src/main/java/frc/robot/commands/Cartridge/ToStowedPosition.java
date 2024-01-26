// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Cartridge;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cartridge;

public class ToStowedPosition extends Command {
  private Cartridge cartridge;

  
  /** Creates a new MoveCartridge. */
  public ToStowedPosition(Cartridge cartridge) {
    this.cartridge = cartridge;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridge);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  cartridge.cartridgeStowedPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;  //*** Try false here if having trouble executing this command
  }
}
