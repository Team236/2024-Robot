// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class ArcadeDrive extends Command {
  private Drive drive;
  private XboxController driveController;
  private DifferentialDrive diffDrive;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drive drive) {
    this.drive = drive;
   // this.diffDrive = diffDrive;
   // this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.openRampRate(); //TODO try without this, also add slew rate
    //drive.resetLeftEncoder();
    //drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arcadeDrive(speed, rotation) - speed is Y-axis of right stick, rotation is x-axis of left stick
    //xspeed is getLeftY, zrotation is getRightX, for arcade drive with 2 joysticks
    if(drive.inYDeadzone() && drive.inXDeadzone()) {
      drive.driveArcade(0,0);
    } else if(!drive.inYDeadzone() && !drive.inXDeadzone()) { 
      drive.driveArcade(-driveController.getRightY(), -driveController.getLeftX());
    } else if(drive.inYDeadzone() && !drive.inXDeadzone()) {
      diffDrive.arcadeDrive( 0, -driveController.getLeftX());
    } else if(!drive.inYDeadzone() && drive.inXDeadzone()) {
      diffDrive.arcadeDrive(-driveController.getRightY(), 0);
    }
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
