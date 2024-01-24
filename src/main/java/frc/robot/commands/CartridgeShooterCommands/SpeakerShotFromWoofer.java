// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeShooterCommands;
import edu.wpi.first.wpilibj2.command.Command;

public class SpeakerShotFromWoofer extends Command {
  

  public SpeakerShotFromWoofer() {
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //***TODO  Add here - reset the counter to zero (do after all shots) - counter is in Robot.Java
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
