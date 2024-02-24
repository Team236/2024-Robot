// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraServo;

public class CameraToggle extends Command {

  private CameraServo cameraServo;

  /** Creates a new CameraToggle. */
  public CameraToggle(CameraServo cameraServo) {
    this.cameraServo = cameraServo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cameraServo);
  }

  @Override
  public void initialize() {
    cameraServo.setIsFloor(true);
  }

  @Override
  public void execute() {
//***assumes camera aimed at floor at start of match (see Robot.java, teleopInit)
  if (cameraServo.isFloor()) { 
    cameraServo.setAngle(Constants.FRONT_CAM_AMP);
    cameraServo.setIsFloor(false);
  } else {
    cameraServo.setAngle(Constants.FRONT_CAM_FLOOR);
    cameraServo.setIsFloor(true);

  }
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
