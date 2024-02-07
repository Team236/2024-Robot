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
  private double tv, distX, errorX;
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    cameraXoffset = 4; //need to figure out
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    distX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    errorX = distX - cameraXoffset;//TODO - determine this offset

    if(tv==1) {
    //Establishes error in the x axis 
      if (Math.abs(errorX)>0.5){
      SmartDashboard.putNumber("Adjust Angle, ErrorX is:", errorX);
        double steeringAdjust = kX * errorX;
        drive.setLeftSpeed(steeringAdjust);//if cam in back, use - here?
        drive.setRightSpeed(-steeringAdjust); //if cam in back, use + here?
        }   
      else{
      SmartDashboard.putNumber("No Shoot Target", tv);
      }
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
    // return false;

    if(tv==1 && Math.abs(errorX)<=2){
      SmartDashboard.putBoolean("LLAngle isFinished:", true);
      return true;
      }   
      else if(tv==1 && Math.abs(errorX)>2){
        return false;
      }
      else
      {
      SmartDashboard.putNumber("No Shoot Target", tv);
      return true;
      }
  }
}
