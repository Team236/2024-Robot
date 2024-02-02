// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cartridge;

public class PIDCartridgeTilt extends Command {

  private Cartridge cartridge;
  private double desiredRevs; //desired height in inches
  private final PIDController pidController;
  private double KP, KI, KD;

  /** Creates a new PIDCartridgeTilt. */
  public PIDCartridgeTilt(Cartridge cartridge, double desiredRevs, double KP, double KI, double KD) {
    this.KP = KP;
    this.KI = KI;
    this.KD = KD;
    pidController = new PIDController(KP, KI, KD);
    this.cartridge = cartridge;
    this.desiredRevs = desiredRevs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cartridge);
    pidController.setSetpoint(desiredRevs);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cartridge.setTiltSpeed(pidController.calculate(cartridge.getTiltEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cartridge.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
