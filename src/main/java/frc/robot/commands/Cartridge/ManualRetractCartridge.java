// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cartridge;

public class ManualRetractCartridge extends Command {
  /** Creates a new ManualExtCartridge. */
  private Cartridge cartridge;
  private double speed;

  public ManualRetractCartridge(Cartridge cartridge, double speed) {
    this.speed = speed;
    this.cartridge= cartridge;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridge);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cartridge.setTiltSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridge.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cartridge.isTRetLimit();
  }
}
