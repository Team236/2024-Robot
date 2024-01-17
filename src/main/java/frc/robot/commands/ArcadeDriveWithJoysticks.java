// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class ArcadeDriveWithJoysticks extends Command {
  /** Creates a new ArcadeDriveWithJoysticks. */
    private Drive drive;

  public ArcadeDriveWithJoysticks(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       //TODO try without ramp rate, also try adding slew rate in driveArcadeWithSticks method
       drive.openRampRate();
       //drive.resetLeftEncoder();
       //drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.ArcadeWithSticks();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}