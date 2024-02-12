// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;


public class LLTarget extends Command {
  
  private double kX = 0.017;//ADJUST!!!  0.005??
  private double kY = 0.03; //0.00725;
  private Drive drive;
  private double h1 = 34; //approx ht now, was 32.5 in 2023 //inches, distance from floor to center of camera lens
  //private double h2 = 18; // inches, same unit as d, to center of target
  private double a1 = Math.toRadians(6); //was 20 degrees in 2023 - camera angle
  private double dist; //desired distance from camera to target - pass into command
  private double steeringAdjust;
  private double cameraXoffset; 
  //private Limelight limelight;
  private double pipeline;
  private double targetHeight;//18" for Atag, from floor to center of target
  private double a2, dx, errorY, distanceAdjust;
  /** Creates a new LLTarget. */
  public LLTarget(Drive drive, double pipeline, double standoff, double targetHeight) {
    this.drive = drive;
    this.pipeline = pipeline;
    this.dist = standoff;
    this.targetHeight = targetHeight;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("LLTarget init", pipeline);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    cameraXoffset = 4; 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
   // TO make sure dx is positive, use abs value for disY and (h1-h2)
    double disY= Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    double disX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double errorX = disX - cameraXoffset; 
    
    if(tv==1){
      if(Math.abs(errorX)>0.5){
        steeringAdjust = (kX * errorX); 
        }
      else {
        steeringAdjust = 0;  
      }
         a2 = disY*Math.PI/180;  //make sure disY is positive
         dx = Math.abs((h1-targetHeight)) / Math.tan(a1+a2);
         errorY = dist - dx;
         distanceAdjust = kY * errorY; 
         
       drive.setLeftSpeed(-distanceAdjust + steeringAdjust);
       drive.setRightSpeed(-distanceAdjust - steeringAdjust); 
       
      SmartDashboard.putNumber("ErrorX - Angle Error tX", errorX);
      SmartDashboard.putNumber("dx, Y dist from target:", dx);
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
/*if(tv==1 && Math.abs(errorY)<=2 && Math.abs(errorX <= 5)){
      SmartDashboard.putBoolean("LLDistance isFinished:", true);
      return true;
      }   
      else if(tv==1 && (Math.abs(errorY) > 2 || Math.abs(errorX > 5) {
         SmartDashboard.putBoolean("LLDistance still working angle or distance", true);
        return false;
      }
      else
      {
      SmartDashboard.putNumber("No Shoot Target", tv);
      return true;
      }
      */
  }
}