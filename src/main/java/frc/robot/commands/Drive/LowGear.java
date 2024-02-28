// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;


public class LowGear extends Command {

  //use "driveLG" not just "drive", so you can drive while changing gears
private Drive drive;
  /** Creates a new LowGear. */
  public LowGear(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
 addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setGearLow();
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
    return true; //gearchange is in intializa, so returning true will release drive subsystem so you can drive
  }
}
