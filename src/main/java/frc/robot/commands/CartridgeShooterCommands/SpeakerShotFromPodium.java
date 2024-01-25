// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CartridgeShooter;

public class SpeakerShotFromPodium extends Command {
  
  private CartridgeShooter cartridgeShooter;
  public Robot robot;
  
  public SpeakerShotFromPodium(CartridgeShooter cartridgeShooter, Robot robot) {
    this.cartridgeShooter = cartridgeShooter;
    this.robot = robot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridgeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     cartridgeShooter.setBothSpeeds(Constants.CartridgeShooter.PODIUM_SHOT_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridgeShooter.setBothSpeeds(0);
    //robot.counter.reset(); //add this after adding TrapAmpScorer branch changes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
