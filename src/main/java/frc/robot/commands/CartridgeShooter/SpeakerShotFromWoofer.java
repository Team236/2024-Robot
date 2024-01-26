// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeShooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CartridgeShooter;
import frc.robot.subsystems.Intake;


public class SpeakerShotFromWoofer extends Command {

private CartridgeShooter cartridgeShooter; 
private Intake intake;

  public SpeakerShotFromWoofer(CartridgeShooter cartridgeShooter, Intake intake) {
    this.cartridgeShooter = cartridgeShooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridgeShooter);
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cartridgeShooter.setBothSpeeds(Constants.CartridgeShooter.WOOFER_SHOT_MOTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the shooter motor and reset Note count to zero, after shooting
      cartridgeShooter.setBothSpeeds(0);
      intake.resetCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
