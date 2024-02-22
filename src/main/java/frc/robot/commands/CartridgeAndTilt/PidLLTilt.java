// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tilt;

public class PidLLTilt extends Command {
  private Tilt tilt;
  private double desiredRevs; //desired height in inches
//Limelight stuff:
    //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair of LL to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target  57.5"?
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //dx = horizontal distance from Limelight Lens to target
    //Dx = dx - offset = horizontal distance from robot bumper to target
    //offset = distance from LL lens to outer edge of bumper
    //tan(a1 +a2)  = (h2-h1)/dx;
  private double h1 = 48;// inches from ground to center of camera lens - MEASURE
  private double h2 = 57.5; // inches, same unit as d, to center of target - GET THIS VALUE
  private double a1 = Math.toRadians(6); //6 degrees, camera tilt
  private double offset = 15; //TODO get actual number
  private double pipeline;
  private double tv, disY, a2, dx, Dx;

  /** Creates a new PidLLTilt. */
  public PidLLTilt(Tilt tilt) {
      this.tilt = tilt;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tilt.setP(Constants.Tilt.KP_TILT);
    tilt.setI(Constants.Tilt.KI_TILT);
    tilt.setD(Constants.Tilt.KD_TILT);
    tilt.setFF(Constants.Tilt.KFF_TILT);
    SmartDashboard.putNumber("LLDistance init", pipeline);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  
     // TO make sure dx is positive, use abs value for disY and (h1-h2)
    disY = Math.abs (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
 
     if(tv==1){
         a2 = disY*Math.PI/180; // in radians, if disY in degrees
         dx = Math.abs(h2 - h1) / Math.tan(a1+a2);  
         SmartDashboard.putNumber("dx, Y dist from target:", dx); //test this - use later for cartridge angle equation
         SmartDashboard.putNumber("Ty, degrees:", disY);
      } else{
         SmartDashboard.putNumber("No Target", tv);
      }

      Dx = dx - offset;
      
      if (Dx < 6) {
      desiredRevs = 16;  //TODO get actual numbers
    } else if  ((Dx >= 6) || (Dx < 12))  {
      desiredRevs = 20;
    } else if  ((Dx >= 12) || (Dx < 18))  {
      desiredRevs = 25;
    } else if  ((Dx >= 18) || (Dx < 24))  {
      desiredRevs = 30;
    } else if  ((Dx >= 24) || (Dx < 30))  {
      desiredRevs = 35;
    } else if  ((Dx >= 36) || (Dx < 42))  {
      desiredRevs = 40;
    } else if  ((Dx >= 48) || (Dx < 54))  {
      desiredRevs = 45;
    } else  {
      desiredRevs = 50;
    }

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
    else if ( (tilt.getTiltSpeed() < 0) && (tilt.isTRetLimit()) ) {
      isAtLimit = true; 
    }
    else isAtLimit = false;
   // SmartDashboard.putBoolean("Tilt isFinished condition is: ", isAtLimit);

    return isAtLimit;
  }
}
