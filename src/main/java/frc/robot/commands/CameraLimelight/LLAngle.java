// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class LLAngle extends Command {
    private double kX = 0.07;
    //0.022;//From pidTurn
  private double distX, tv, errorX;
  private Drive drive;
  private double pipeline, cameraXoffset;

  public LLAngle(Drive drive, double pipeline) {
      this.drive = drive;
      this.pipeline = pipeline;
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("LLangle init", pipeline);
    // turn on the if set for pipeline: 0 is "controlled by pipeline"
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    cameraXoffset = 0; //check if LL camera not centered on robot (left to right)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // turn off the LED  1 = "force off" 0 = "controlled by pipeline", 3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    distX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    errorX = distX - cameraXoffset;

    if(tv==1) {   
    //Establishes error in the x axis 
      if (Math.abs(errorX)>0.5){
      SmartDashboard.putNumber("Adjust Angle, ErrorX is:", errorX);
        double steeringAdjust = kX * errorX;
        drive.setLeftSpeed(steeringAdjust);
        drive.setRightSpeed(-steeringAdjust); 
        }   
      else{
      // SmartDashboard.putNumber("LL tag Id", tid);
      SmartDashboard.putNumber("No Shoot Target:", tv);
      }
     }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // turn off the LED  1 = "force off" 0 = "controlled by pipeline"
   // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(tv==1 && Math.abs(errorX)<=1){
        SmartDashboard.putBoolean("LLAngle isFinished:", true);
        return true;
        }   
    else if(tv==1 && Math.abs(errorX)>1){
        return false;
        }
    else {
         SmartDashboard.putNumber("No Shoot Target", tv);
         return true;
        }
  
   /*  if(tid != 0 && Math.abs(errorX)<=2){
      SmartDashboard.putBoolean("LLAngle isFinished:", true);
      // turn off the LED  1 = "force off" 
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); 
      return true;
      }   
      else if(tid != 0 && Math.abs(errorX)>2){
        return false;
      }
      else
      {
      
      SmartDashboard.putNumber("No Shoot Target", 1);
      SmartDashboard.putNumber("LL tag Id", tagId);
      // turn off the LED  1 = "force off" 
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); 
      return true;
      }
      */
  }
    
}

