// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilt;

public class ManualExtCartridge extends Command {
  //Extends the cartridge manually, stopping at the limit or max extend value
  private Tilt tilt;
  private double speed;

  public ManualExtCartridge(Tilt tilt, double speed) {
    this.speed = speed;
    this.tilt = tilt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tilt);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.setTiltSpeed(-speed);   //WAS POSITIVE
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tilt.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
