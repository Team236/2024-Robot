// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ToggleCameraAngle extends Command {
private boolean toggle;

  /** Creates a new ToggleCameraAngle. */
  public ToggleCameraAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //if angle more than 90 degrees (0.5), it is set to see Floor, so reset it to see Amp
  //otherwise it is set to see the Amp, so reset it to see the Floor
  //comand is finised when toggle is true 
  toggle = false;

  if (Robot.cameraServo.get() > 0.5) {  //TODO - FIND TRUE VALUE HERE FOR AMP vs FLOOR ANGLES
    Robot.cameraServo.set(Constants.FRONT_CAM_AMP);
  } else {
    Robot.cameraServo.set(Constants.FRONT_CAM_FLOOR);
  }

  toggle = true;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toggle;
  }
}
