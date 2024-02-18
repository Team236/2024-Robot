// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class TankJoysticks extends Command {
  /** Creates a new TankDriveWithJoysticks. */
  private Drive drive;
  private boolean inLeftYDeadzone, inRightYDeadzone;
  private DifferentialDrive diffDrive;
  private double leftSpeed, rightSpeed;
  private Joystick leftStick, rightStick;

  public TankJoysticks(DifferentialDrive diffDrive, Joystick leftStick, Joystick rightStick, Drive drive) {
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
   // drive.openRampRate();
    //TODO try without ramp rate, also try adding slew rate in driveTank method
    //drive.resetLeftEncoder();
    //drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //tankDrive(YleftSpeed, YrightSpeed))
    /*
    inLeftYDeadzone = Math.abs(leftStick.getY()) <= Constants.DriveConstants.LEFT_DEADZONE;
    inRightYDeadzone = Math.abs(rightStick.getY()) <= Constants.DriveConstants.RIGHT_DEADZONE;
     if(inLeftYDeadzone && inRightYDeadzone) {
      leftSpeed =0;
      rightSpeed = 0;
     } else if(!inLeftYDeadzone && !inRightYDeadzone) { 
       leftSpeed = -leftStick.getY();
       rightSpeed = -rightStick.getY();
     } else if(inLeftYDeadzone && !inRightYDeadzone) {
       leftSpeed = 0; 
       rightSpeed = -rightStick.getY();
     } else if(!inLeftYDeadzone && inRightYDeadzone) {
      leftSpeed = -leftStick.getY(); 
      rightSpeed =0;
   }
   */
    //TODO  - replace last line in execute with the 2 lines below and see effect
    //SlewRateLimiter filter = new SlewRateLimiter(0.5);
    // limits the rate of change of the signal to 0.5 units per second
    //diffDrive.tankDrive(filter.calculate(leftSpeed), filter.calculate(rightSpeed);
      leftSpeed = -leftStick.getY();
      rightSpeed = -rightStick.getY();

      diffDrive.tankDrive(leftSpeed, rightSpeed);
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
