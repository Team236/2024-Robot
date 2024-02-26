// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
    cameraXoffset = 0; //TODO - change if camera not centered
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   Pose3d cameraTargetPose = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
    distX = cameraTargetPose.getX();  // camera position from tag
    errorX = distX - cameraXoffset;//TODO - determine this offset

    if(LimelightHelpers.getFiducialID("limelight")!= 0 && Math.abs(errorX) < 0.5 ) {
    //Establishes error in the x axis 
      // if {
      SmartDashboard.putNumber("Adjust Angle, ErrorX is:", errorX);
      double steeringAdjust = kX * errorX;
    //****IF TESTING WITH 2023 ROBOT, CHANGE LEFT TO POSITIVE, RIGHT TO NEGATIVE SINCE CAM IN FRONT *********//
        drive.setLeftSpeed(-steeringAdjust);//since cam in back, use - here (was + in 2023)
        drive.setRightSpeed(+steeringAdjust); //since cam in back, use + here (was - in 2023)
        }   
      else{
      SmartDashboard.putNumber("No Shoot Target", tv);
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
