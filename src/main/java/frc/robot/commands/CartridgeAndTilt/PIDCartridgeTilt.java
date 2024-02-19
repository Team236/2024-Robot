// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Tilt;

public class PIDCartridgeTilt extends Command {

  private Tilt tilt;
  private double desiredRevs; //desired height in inches

  /** Creates a new PIDCartridgeTilt. */
 public PIDCartridgeTilt(Tilt tilt, double desiredRevs) {
    this.tilt = tilt;
    this.desiredRevs = desiredRevs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tilt);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pidController.reset();
    tilt.setP(Constants.Tilt.KP_TILT);
    tilt.setI(Constants.Tilt.KI_TILT);
    tilt.setD(Constants.Tilt.KD_TILT);
    tilt.setFF(Constants.Tilt.KFF_TILT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tilt.setSetpoint(desiredRevs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tilt.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isAtLimit;
    if ( (tilt.getTiltSpeed() > 0)  && (tilt.isTExtLimit() || tilt.isFullyExtended()) ) {
      isAtLimit = true;
    } 
    else if ( (tilt.getTiltSpeed() <= 0) && (tilt.isTRetLimit()) ) {
      isAtLimit = true; 
    }
    else isAtLimit = false;

    return isAtLimit;
    }

  }

