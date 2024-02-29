// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class ToggleGear extends Command {
  private Drive drive;
  private boolean toggle;
  private boolean isLow;
  /** Creates a new ToggleGear. */
  public ToggleGear(Drive drive) {
  this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //if in low gear change to high gear and set toggle true
  //otherwise change to low gear and set toggle true
  //comand is finised when toggle is true 
  toggle = false;

  if (drive.isInLowGear()) {
    drive.setGearHigh();
    isLow = false;
  } else {
    drive.setGearLow();
    isLow = true;
  }
  
  toggle = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("In High Gear?:  ",!isLow);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toggle;
  }
}
