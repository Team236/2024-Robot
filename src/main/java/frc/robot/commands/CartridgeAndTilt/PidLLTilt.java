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
  private double h1 = 44;// inches from ground to center of camera lens
  private double h2 = 57.5; // inches,floor to center of target
  private double a1 = Math.toRadians(10); //degrees, camera tilt, up from horizontal
  private double offset = 7; //inhces, LL lens to outer edge of bumper
  private double pipeline;
  private double tv, angleY, a2, dx, Dx;

  /** Creates a new PidLLTilt. */
  public PidLLTilt(Tilt tilt, double pipeline) {
      this.tilt = tilt;
      this.pipeline = pipeline;
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
    //angle between trage and LL camera lens is ty:
    angleY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
 
     if(tv==1){
         a2 = Math.toDegrees(angleY); //same as multiplying times PI/180; 
         dx = (h2 - h1) / Math.tan(a1+a2);  
         SmartDashboard.putNumber("LLdx, distance from target:", dx); //test this - use later for cartridge angle equation
         SmartDashboard.putNumber("LLDx, dist woofer to bumper: ", dx-36-offset);
         SmartDashboard.putNumber("LLty, degrees:", angleY);
      } else{
         SmartDashboard.putNumber("No Target", tv);
      }
      Dx = dx - 36 - offset;
      
      if (Dx < 6) {
      desiredRevs = 16;  //TODO get actual desiredRevs numbers
    } else if  ((Dx >= 6) || (Dx < 12))  {
      desiredRevs = 18.4;
    } else if  ((Dx >= 12) || (Dx < 18))  {
      desiredRevs = 20.8;
    } else if  ((Dx >= 18) || (Dx < 24))  {
      desiredRevs = 23.1;
    } else if  ((Dx >= 24) || (Dx < 30))  {
      desiredRevs = 25.5;
    } else if  ((Dx >= 30) || (Dx < 36))  {
      desiredRevs = 27.8;
    } else if  ((Dx >= 36) || (Dx < 42))  {
      desiredRevs = 30.2;
    } else if  ((Dx >= 42) || (Dx < 48))  {
      desiredRevs = 32.6;
    } else if  ((Dx >= 48) || (Dx < 54))  {
      desiredRevs = 35;
    } else  {
      desiredRevs = 37.4;
    }
    tilt.setSetpoint(desiredRevs);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
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
