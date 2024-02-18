// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class CurvatureXbox extends Command {
    private Drive drive;
  private XboxController driverController;
  private DifferentialDrive diffDrive;
   // public CANSparkMax leftFront, rightFront;
  private double leftSpeed, rightRotation;
  /** Creates a new CurvatureXbox. */
  public CurvatureXbox(DifferentialDrive diffDrive, XboxController driverController, Drive drive) {
    this.drive = drive;
    this.driverController = driverController;
    this.diffDrive = diffDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO try without ramp rate, also try adding slew rate in driveArcade method
    //drive.openRampRate();
    //drive.resetLeftEncoder();
    //drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //TODO  - replace last line in execute with the 2 lines below and see effect
  //SlewRateLimiter filter = new SlewRateLimiter(0.5);
  //diffDrive.arcadeDrive(filter.calculate(leftSpeed), rightRotation);
  // (limits the rate of change of the signal to 0.5 units per second)
  
  leftSpeed = -driverController.getLeftY();
  rightRotation = -driverController.getRightX();

  diffDrive.setDeadband(Constants.DriveConstants.DEADBAND);
  diffDrive.curvatureDrive(leftSpeed, rightRotation, driverController.getAButton());
  //more like car steering, rightX controls curvature of rotation
  //when AButton pressed, it makes the robot turn in-place
  //button 1 on the XBox controller is "A" - link to a command in Container later)
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
