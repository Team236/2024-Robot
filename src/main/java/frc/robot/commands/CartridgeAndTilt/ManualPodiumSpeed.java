// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;

public class ManualPodiumSpeed extends Command {
  //runs cartridge at a set speed, podium shot speed without PID
  
  private Cartridge cartridge;
  
  public ManualPodiumSpeed(Cartridge cartridge) {
    this.cartridge = cartridge;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridge);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //cartridge.podiumShotPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     cartridge.setBothSpeeds(Constants.CartridgeShooter.PODIUM_SHOT_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the shooter motor and reset the Note count to zero, after shooting
    cartridge.setBothSpeeds(0);
    Intake.resetCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
