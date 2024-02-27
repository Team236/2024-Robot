// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class CameraToggle extends Command {
  /** Creates a new CameraToggle. */
  public CameraToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    //for solenoids, just do action once, not in execute, then command ends
    //must start match with camera aimed at floor (isFloor set to True in Robot.java)
    if (Robot.isFloor) { 
    Robot.cameraServo.set(Constants.FRONT_CAM_TRAP);
    Robot.isFloor = false;
  } else {
    Robot.cameraServo.set(Constants.FRONT_CAM_TELEOP);
    Robot.isFloor = true;
  }
  }

  @Override
  public void execute() {
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
