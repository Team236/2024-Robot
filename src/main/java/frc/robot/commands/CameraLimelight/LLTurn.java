// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class LLTurn extends Command {
  private double KpLeft; //TODO tune each side individually
  private double KpRight ;//0.0069; //TODO tune each side individually
  private double angleXError, tv, errorX;
  private Drive drive;
  private double pipeline = 0, cameraXoffset;

  public LLTurn(Drive drive, double pipeline) {
      this.drive = drive;
      this.pipeline = pipeline;
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   //SmartDashboard.putNumber("LLangle init", pipeline);
    // 0 is "controlled by pipeline"
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    cameraXoffset = 0; //check if LL camera not centered on robot (left to right)

    if (drive.isInHighGear()) {
        KpLeft = 0.006;//0.0027; //TODO tune each side individually
        KpRight = 0.006;//0.0027;
     } else {
        KpLeft = 0.007;//0.005; //.0069//TODO tune each side individually
        KpRight = 0.007;//0.005
     }
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    angleXError = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    errorX = angleXError - cameraXoffset;  //degrees

    if(tv==1) {   
        //SmartDashboard.putNumber("Adjust Angle:", errorX);
        double steeringAdjustLeft = KpLeft * errorX;
        double steeringAdjustRight = KpRight * errorX;
        drive.setLeftSpeed(steeringAdjustLeft);
        drive.setRightSpeed(-steeringAdjustRight); 
        }   
     // else{
     //SmartDashboard.putNumber("No Shoot Target:", tv);
     // }
     }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // turn off the LED, 0 = "controlled by pipeline"
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    //TODO - try reducing max error
    if(tv==1 && Math.abs(errorX)<=2){
        SmartDashboard.putBoolean("LLAngle isFinished:", true);
        return true;
        }   
    else if(tv==1 && Math.abs(errorX)>2){
        return false;
        }
    else {
         //SmartDashboard.putNumber("No Shoot Target", tv);
         return true;
        }
  }
}
