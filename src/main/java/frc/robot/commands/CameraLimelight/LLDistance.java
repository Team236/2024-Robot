// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
  
  
  
  private Drive drive;
  private int pipeline;
  private double standoff, disY, dx, errorY;
  private double[] limeTarget;
  private double tag_id = 0;
  
  /** CONSTRUCTOR - Creates a new LLAngle. */
  public LLDistance(Drive drive, int pipeline, double standoff) {
    this.drive = drive;
    this.pipeline = pipeline;
    this.standoff = standoff;

    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeTarget = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    SmartDashboard.putNumber("LLDistance init", pipeline);
    SmartDashboard.putNumberArray("tag_target_array",LimelightHelpers.getTargetPose_RobotSpace("limelight"));
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setPipelineIndex("limelight",pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // this is already initialized to off might  be redundant
   LimelightHelpers.setLEDMode_ForceOff("limelight");
    // could be faster to return limelight x,y,z,xr,yr,zr 'Results' all at once
    // this gets pose but not tag_id and all other data at once
   limeTarget = LimelightHelpers.getTargetPose_RobotSpace("limelight"); 
   
   // there is no "has target=tv " for the april tags but current tag_id should be null if not found
   //tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
   tag_id = LimelightHelpers.getFiducialID("limelight");
 
    // TO make sure dx is positive, use abs value for disY and (h1-h2)
  //  disY = Math.abs (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));

   // if has_target should change to switch case with tag_id
   // alternative is if then statement tag_id > 0 do something 

   switch ((int)tag_id) {
    case 1,2: 
    // do samething for the 1,2 Red Blue Amp
    driveDistance(Constants.Amp.AMP_TAG_DISTANCE);
    break;
    case 3, 4:
      // do something for the 3,4 Red speaker 
      // Do we want to declare standoff values in command use or via set constants
      if (standoff==0) { driveDistance(Constants.Speaker.SPEAKER_TAG_DISTANCE);
      } else  driveDistance(standoff);
      break;
    case 7, 8:
      // do something for the 7,8 Blue speaker
      if (standoff==0) { driveDistance(Constants.Speaker.SPEAKER_TAG_DISTANCE);
      } else  driveDistance(standoff);
      break;
    case 11,12,13:     // do something for the 11,12,13 Blue stage
        // falls through to to next break if 'break' is missing
    case 14,15,16:
      // do something for the 14,15,16 Red stage
      if (standoff==0) { driveDistance(Constants.Speaker.SPEAKER_TAG_DISTANCE);
      } else  driveDistance(standoff);
      break;
    default:   // default used undeclared or nothing if no default action
      break;      
   }
  }  
    
   private void driveDistance(double distance) {
   if(tag_id>0){
        dx = limeTarget[2];       // TODO verify position 2 Y-distance is correct axis for 'TargetPose_RobotSpace'   
        errorY = distance - dx;  
    //NOTE:  CAN TRY TO USE THE Z VALUE OF THE POSE FOR errorY (use [2] or [0] for other directions)
      double distanceAdjust = kY * errorY;
       drive.setLeftSpeed(-distanceAdjust);   //change to negative since LL is in back of robot this year
       drive.setRightSpeed(-distanceAdjust);  //change to negative since LL is in back of robot this year
      SmartDashboard.putNumber("dx, Y dist from target:", dx); //test this - use later for cartridge angle equation
      SmartDashboard.putNumber("ErrorY:", errorY);
      SmartDashboard.putNumber("Ty, degrees:", disY);
   } else{
      SmartDashboard.putNumber("No Target", tag_id);
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
    if(Math.abs(errorY)<=1){
      SmartDashboard.putBoolean("LLDistance isFinished:", true);
      return true;
      }   
      else if(Math.abs(errorY)>1){
        return false;
      }
      else
      {
      return true;
      }
}
  }