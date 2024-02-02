// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Camera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.AmpTrap;

public class FloorCameraAngle extends Command {
//Moves the camera servo to view the Amp or Trap
//The servo goes from 0 to 1, for 0 to 180 degrees.
private AmpTrap ampTrapShooter;

  /** Creates a new FloorCameraAngle. */
  public FloorCameraAngle(AmpTrap ampTrapShooter) {
    this.ampTrapShooter = ampTrapShooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.ampTrapShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.frontCamServo.set(Constants.FRONT_CAM_FLOOR);
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
