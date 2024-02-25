// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class LLAngle extends Command {
    private double kX = 0.017;//TODO - try different kX
  private double tid, distX, errorX;
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
    // turn on the if set for pipeline:  0 is "controlled by pipeline"
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    cameraXoffset = 0; //TODO - change if camera not centered on robot (left to right)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turn off the LED  1 = "force off" 0 = "controlled by pipeline"
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    distX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    errorX = distX - cameraXoffset;//TODO - determine this offset

    if(tid != 0) {   
    //Establishes error in the x axis 
      if (Math.abs(errorX)>0.5){
      SmartDashboard.putNumber("Adjust Angle, ErrorX is:", errorX);
        double steeringAdjust = kX * errorX;
    //****IF TESTING WITH 2023 ROBOT, CHANGE LEFT TO POSITIVE, RIGHT TO NEGATIVE SINCE CAM IN FRONT *********//
        drive.setLeftSpeed(-steeringAdjust);//since cam in back, use - here (was + in 2023)
        drive.setRightSpeed(+steeringAdjust); //since cam in back, use + here (was - in 2023)
        }   
      else{
      // SmartDashboard.putNumber("LL tag Id", tid);
      SmartDashboard.putNumber("No Shoot Target", 1);  // 1 is false
      }
     }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // turn off the LED  1 = "force off" 0 = "controlled by pipeline"
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;

    if(tid != 0 && Math.abs(errorX)<=2){
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
  }
    
}
