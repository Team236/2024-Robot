// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;

public class LLDistance extends Command {
    //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset from crosshair to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical?)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //d = Distance to target (want 14" or 16" distance in order to be in front of Grid)
    //tan(a1 +a2)  = (h2-h1)/dx;

  private double kY = 0.02; //0.00725;
  
  private double h1 = 34;// approx ht now = was 32.5; //inches, from ground to center of camera lens
  //private double h2 = 18; // inches, same unit as d, to center of target
  private double a1 = Math.toRadians(6); //6 degrees, camera tilt
  private double dist; // desired distance from camera to target in inches; pass into command
  private Drive drive;
  private double pipeline;
  private double targetHeight;//18" for Atag, from floor to center of target
  private double tv, disY, a2, dx, errorY;
  
  /** Creates a new LLAngle. */
  public LLDistance(Drive drive, double pipeline, double standoff, double targetHeight) {
    this.drive = drive;
    this.pipeline = pipeline;
    this.dist = standoff;
    this.targetHeight = targetHeight;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
        dx = Math.abs(targetHeight - h1) / Math.tan(a1+a2);  
        errorY = dist - dx;  
    //NOTE:  CAN TRY TO USE THE Z VALUE OF THE POSE FOR errorY (use [2] or [0] for other directions)
      double distanceAdjust = kY * errorY;
       drive.setLeftSpeed(-distanceAdjust); //negative since LL is in back of robot this year
       drive.setRightSpeed(-distanceAdjust);  //negative since LL is in back of robot this year
      SmartDashboard.putNumber("dx, Y dist from target:", dx); //test this - use later for cartridge angle equation
      SmartDashboard.putNumber("ErrorY:", errorY);
      SmartDashboard.putNumber("Ty, degrees:", disY);
   } else{
      SmartDashboard.putNumber("No Target", tv);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if(tv==1 && Math.abs(errorY)<=1){
      SmartDashboard.putBoolean("LLDistance isFinished:", true);
      return true;
      }   
      else if(tv==1 && Math.abs(errorY)>1){
        return false;
      }
      else
      {
      SmartDashboard.putNumber("No Shoot Target", tv);
      return true;
      }
      
}
  }