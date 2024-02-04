// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Tilt;

public class PIDCartridgeTilt extends Command {

  private Tilt tilt;
  private double desiredRevs; //desired height in inches
  private final PIDController pidController;
  private double KP, KI, KD;

  /** Creates a new PIDCartridgeTilt. */
  public PIDCartridgeTilt(Tilt tilt, double desiredRevs, double KP, double KI, double KD) {
    this.KP = KP;
    this.KI = KI;
    this.KD = KD;
    pidController = new PIDController(KP, KI, KD);
    this.tilt = tilt;
    this.desiredRevs = desiredRevs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tilt);
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
    tilt.setTiltSpeed(pidController.calculate(tilt.getTiltEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tilt.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
