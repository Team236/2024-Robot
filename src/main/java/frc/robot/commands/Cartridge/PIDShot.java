// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;

public class PIDShot extends Command {
  private Cartridge cartridge;
  private double speed;

  /** Creates a new PIDShot. */
  public PIDShot(Cartridge cartridge, double speed) {

    this.cartridge = cartridge;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.cartridge);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cartridge.resetEncoders();

    cartridge.setP(Constants.CartridgeShooter.kPLeft, Constants.CartridgeShooter.kPRight);
    cartridge.setI(Constants.CartridgeShooter.kILeft, Constants.CartridgeShooter.kIRight);
    cartridge.setD(Constants.CartridgeShooter.kDLeft, Constants.CartridgeShooter.kDRight);
    cartridge.setFF(Constants.CartridgeShooter.kFFLeft, Constants.CartridgeShooter.kFFRight);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    cartridge.setOutputRange();
    cartridge.setSetpoint(speed);

   // SmartDashboard.putNumber("Speed setpoint is ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridge.setBothSpeeds(0);
    Intake.resetCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
