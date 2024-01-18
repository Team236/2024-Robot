// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ArcadeXbox extends Command {
  private Drive drive;
  private XboxController driverController;
  private boolean inLeftYDeadzone, inRightXDeadzone;
  private DifferentialDrive diffDrive;
    public CANSparkMax leftFront, rightFront;
  private double leftSpeed, rightRotation;

  // Creates a new ArcadeXbox 
  //for driving Arcade style using 2 sticks (left/right) on a single XBox 
  public ArcadeXbox(DifferentialDrive diffDrive, XboxController driverController, Drive drive)  {
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
    drive.openRampRate();
    //drive.resetLeftEncoder();
    //drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  inLeftYDeadzone = (Math.abs(driverController.getLeftY()) <= Constants.DriveConstants.LEFT_DEADZONE);
  inRightXDeadzone = (Math.abs(driverController.getRightX()) <= Constants.DriveConstants.RIGHT_DEADZONE);

    //speed is Y-axis of left stick, rotation is X-axis of right stick
    //speed is getLeftY, rotation is getRightX, for arcade drive 
    if(inLeftYDeadzone && inRightXDeadzone){
      leftSpeed =0;
      rightRotation = 0;
    } else if(!inLeftYDeadzone && !inRightXDeadzone) { 
      leftSpeed = -driverController.getLeftY();
      rightRotation = -driverController.getRightX();
    } else if(inLeftYDeadzone && !inRightXDeadzone) {
      leftSpeed = 0;
      rightRotation = -driverController.getRightX();
    } else if(!inLeftYDeadzone && inRightXDeadzone) {
      leftSpeed = -driverController.getLeftY();
      rightRotation = 0;
  }
  diffDrive.arcadeDrive(leftSpeed, rightRotation);
}

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
