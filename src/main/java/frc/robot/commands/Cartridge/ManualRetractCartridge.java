// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilt;

public class ManualRetractCartridge extends Command {
  //Retracts the cartridge manually, stopping at the limit
  private Tilt tilt;
  private double speed;

  public ManualRetractCartridge(Tilt tilt, double speed) {
    this.speed = speed;
    this.tilt= tilt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tilt);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    tilt.setTiltSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tilt.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tilt.isTRetLimit();
  }
}