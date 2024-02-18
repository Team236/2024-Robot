// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ArcadeJoysticks extends Command {
  /** Creates a new ArcadeDriveWithJoysticks. */
    private Drive drive;
    private DifferentialDrive diffDrive;
    private boolean inLeftYDeadzone, inRightXDeadzone;
    private double leftSpeed, rightRotation, leftRotation;
    private Joystick leftStick, rightStick;

  public ArcadeJoysticks(DifferentialDrive diffDrive, Joystick leftStick, Joystick rightStick, Drive drive) {
    this.drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    this.diffDrive = diffDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
       //TODO try without ramp rate, also try adding slew rate in driveArcade method
       //drive.openRampRate();
       drive.resetLeftEncoder();
       drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*
  inLeftYDeadzone = (Math.abs(leftStick.getY()) <= Constants.DriveConstants.LEFT_DEADZONE);
  inRightXDeadzone = (Math.abs(rightStick.getX()) <= Constants.DriveConstants.RIGHT_DEADZONE);

  //arcadeDrive(speed, rotation) - speed is Y-axis of left stick, rotation is x-axis of right stick
  //speed is getLeftY, rotation is getRightX, for arcade drive with 2 sticks
  if(inLeftYDeadzone && inRightXDeadzone) {
   leftSpeed = 0;
   rightRotation = 0; 
  } else if(!inLeftYDeadzone && !inRightXDeadzone) { 
    leftSpeed = -leftStick.getY();
    rightRotation = -rightStick.getX();
  } else if(inLeftYDeadzone && !inRightXDeadzone) {

     leftSpeed = 0;
    rightRotation = -rightStick.getX();
  } else if(!inLeftYDeadzone && inRightXDeadzone) {
     leftSpeed = -leftStick.getY();
    rightRotation = 0;
}
 */
  leftSpeed = -leftStick.getY();
  //leftRotation = -leftStick.getZ(); TODO get rid of this?
  rightRotation = -rightStick.getX();

  diffDrive.arcadeDrive(leftSpeed, rightRotation);

  //TODO  - replace last line in execute with the 2 lines below and see effect
  //SlewRateLimiter filter = new SlewRateLimiter(0.5);
  // limits the rate of change of the signal to 0.5 units per second
  //diffDrive.arcadeDrive(filter.calculate(leftSpeed), rightRotation);
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
