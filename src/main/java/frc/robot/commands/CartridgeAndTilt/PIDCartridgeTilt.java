// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Tilt;

public class PIDCartridgeTilt extends Command {

  private Tilt tilt;
  private double desiredRevs; //desired height in inches
  private final PIDController pidController;
  private double kP = Constants.Tilt.KP_TILT;
  private double kI = Constants.Tilt.KI_TILT;
  private double kD = Constants.Tilt.KD_TILT;

  /** Creates a new PIDCartridgeTilt. */
 public PIDCartridgeTilt(Tilt tilt, double desiredRevs) {
    pidController = new PIDController(kP, kI, kD);
    this.tilt = tilt;
    this.desiredRevs = desiredRevs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tilt);
   pidController.setSetpoint(desiredRevs); //NEED TO MOVE TO EXECUTE SECTION?
   SmartDashboard.putNumber("Setpoint in PIDCartTilt is:  ", desiredRevs);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    SmartDashboard.putString("PIDCartTilt finished init:  ", "true");
    //tilt.setP(Constants.Tilt.KP_TILT); //old code when using SparkMax PID
    //tilt.setI(Constants.Tilt.KI_TILT);
    //tilt.setD(Constants.Tilt.KD_TILT);
    //tilt.setFF(Constants.Tilt.KFF_TILT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //tilt.setSetpoint(desiredRevs); //old code when using SparkMax PID
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
    boolean isAtLimit;
    //WAS >0 below, changed since now encoder is going negative when extending (0 at stow)
    if ( (tilt.getTiltSpeed() < 0)  && (tilt.isTExtLimit() || tilt.isFullyExtended()) ) {  
      isAtLimit = true;
    } 
    //WAS < 0 below, changed since now encoder going positive when retracting
    else if ( (tilt.getTiltSpeed() > 0) && (tilt.isTRetLimit()) ) {
      isAtLimit = true; 
    }
    else isAtLimit = false;
   // SmartDashboard.putBoolean("Tilt isFinished condition is: ", isAtLimit);

    return isAtLimit;
    }

  }

