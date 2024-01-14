// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ArcadeDrive extends Command {
  private Drive drive;
  private XboxController driveController;
  private DifferentialDrive diffDrive;
  private double max, L, R, sensitivityX, sensitivityY;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drive drive, XboxController driveController) {
    this.drive = drive;
    //this.diffDrive = diffDrive;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.openRampRate(); //TODO try without this
    //drive.resetLeftEncoder();
    //drive.resetRightEncoder();
    sensitivityX = Constants.DriveConstants.SENSITIVITY_X;
    sensitivityY = Constants.DriveConstants.SENSITIVITY_Y;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean inDeadzoneX = Math.abs(driveController.getLeftX()) < Constants.DriveConstants.LEFT_DEADZONE;
    boolean inDeadzoneY = Math.abs(driveController.getRightY()) < Constants.DriveConstants.RIGHT_DEADZONE;
    boolean notInDeadzoneX = Math.abs(driveController.getLeftX()) > Constants.DriveConstants.LEFT_DEADZONE;
    boolean notInDeadzoneY = Math.abs(driveController.getRightY()) > Constants.DriveConstants.RIGHT_DEADZONE;
    //DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);
    //drive.arcadeDrive(-rightStick.getY(), leftStick.getX());
    //xspeed is getLeftY, zrotation is getRightX, for arcade drive with 2 joysticks
    if(inDeadzoneX && inDeadzoneY) {
      diffDrive.arcadeDrive(0,0);
    } else if(notInDeadzoneX && notInDeadzoneY) { 
      diffDrive.arcadeDrive(driveController.getLeftX(), driveController.getRightY());
    } else if(inDeadzoneX && notInDeadzoneY) {
      diffDrive.arcadeDrive(0, driveController.getRightY());
    } else if(notInDeadzoneX && inDeadzoneY) {
      diffDrive.arcadeDrive(driveController.getLeftX(), 0);
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
