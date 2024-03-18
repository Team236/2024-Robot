// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Tilt;

public class PidLLTilt extends Command {
  private Tilt tilt;
  private double desiredRevs; //desired encoder revs for the tilt
//Limelight stuff:
    //tv = 1 if there are any targets found, =0 if not
    //h1 = distance from floor to center of Limelight lens is 43.5
    //offset = distance from LL lens to outer edge of bumper    

  private double pipeline;
  private boolean tv;

  
  private double dx, dy, tagDistance; //horizontal distance from AprilTag (target) to LL camera lens
  private double bumperOffset; // distance from edge of bumper to Woofer
   private double cameraOffset = 7.5; //inches, LL lens to outer edge of bumper

  private final PIDController pidController;
  private double kP = Constants.Tilt.KP_TILT;
  private double kI = Constants.Tilt.KI_TILT;
  private double kD = Constants.Tilt.KD_TILT;

  /** Creates a new PidLLTilt. */
  public PidLLTilt(Tilt tilt, double pipeline) {
      pidController = new PIDController(kP, kI, kD);
      this.tilt = tilt;
      this.pipeline = pipeline;

      // does not look like helper can set multiple tags in priority assignment
      //LimelightHelpers.setPriorityTagID("limelight", tagID )
      
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    //tilt.setP(Constants.Tilt.KP_TILT);
    //tilt.setI(Constants.Tilt.KI_TILT);
    //tilt.setD(Constants.Tilt.KD_TILT);
    //tilt.setFF(Constants.Tilt.KFF_TILT);
  

    SmartDashboard.putNumber("LLDistance init", pipeline);
    
          // was NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      LimelightHelpers.setLEDMode_ForceOff("limelight");
          // was NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
      LimelightHelpers.setPipelineIndex("limelight",(int)pipeline); 
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = LimelightHelpers.getTV("limelight");
    double tagId = LimelightHelpers.getFiducialID("limelight");

     if(tagId == 4 || tagId == 7 ) {   // this could also be get gettagId != 0 
 
        double[] targetCameraPose = LimelightHelpers.getCameraPose_TargetSpace("limelight");
        // assumption x target is distance out the from of robot with Limelighthelper
        dx = Units.metersToInches( targetCameraPose[0] );  
        dy = Units.metersToInches( targetCameraPose[1] );
        // z= sqrt(x^2 + y^2)
        tagDistance = Math.sqrt((dx * dx) + (dy * dy));
        Units.metersToInches( tagDistance );
      } else {
         SmartDashboard.putNumber("April tag ID", tagId);
         SmartDashboard.putNumber("camera to tag", 0 );
      }
  
      bumperOffset = tagDistance - 36 - cameraOffset;  //From edge of bumper to woofer 

    //All desiredRevs changed from pos to negative, since tilt motor not inverted
    //So encoder rotations are negative when extending, positive when retracting

 if (bumperOffset < 45.9) {   //old Dx < 3
      desiredRevs = -19;// -17;  //TODO get actual desiredRevs numbers
        } else if  ((dx >= 45.9) && (dx < 50))  {  //old Dx between 3 and 6
      desiredRevs = -21;//-23.6;
    } else if  ((dx >= 50) && (dx < 56.6))  { //old Dx between 6 and 12
      desiredRevs = -22.6;// -23.6;
    } else if  ((dx >= 56.6) && (dx < 62.2))  { //old Dx between 12 and 18
      desiredRevs = -30;//-27.8;
    } else if  ((dx >= 62.2) && (dx < 69))  { //old Dx between 18 and 24
      desiredRevs = -35;//-30;
    } else if  ((dx >= 69) && (dx < 71.85))  { //old Dx between 24 and 30
      desiredRevs = -39;//-32.5;
    } else if  ((dx >= 71.85) && (dx < 76.6))  { //old Dx between 30 and 36
      desiredRevs = -42.3;// -36.85;
    } else if  ((dx >= 76.6) && (dx < 78.3))  { //old Dx between 36 and 39
      desiredRevs = -43.6;//-38.2;
    } else if  ((dx >= 78.3) && (dx < 80))  { //old Dx between 39 and 42
      desiredRevs = -45;//-39.42;
    } else if  ((dx >= 80) && (dx < 81.9))  { //old Dx between 42 and 45
      desiredRevs = -46;// -41.26;
    } else if ((dx >= 81.9) && (dx < 82.9)) { //old Dx between 45 and 48
      desiredRevs = -46.8;//-44;
    } else if ((dx >= 82.9) && (dx < 85.8)) {  //old Dx between 48 and 52 - MEASURE ENC VALUE HERE
      desiredRevs =  -47.2;//-45;  //THIS NEEDS TO BE MEASURED!
    } else if ((dx >= 85.8) && (dx < 86.9)) { //Old Dx between 52 and 55- MEASURE ENC VALUE HERE
      desiredRevs = -48;  //THIS NEEDS TO BE MEASURED
    } else  {
      desiredRevs = -49; //THIS NEEDS TO BE MEASURED - PLUS GO FURTHER OUT THAN 55" from bumber to woofer
    }

    //SmartDashboard.putNumber("Desired Revs", desiredRevs);
    
    pidController.setSetpoint(desiredRevs);  //****moved here, was up top before
    tilt.setTiltSpeed(pidController.calculate(tilt.getTiltEncoder()));
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    tilt.stopTilt();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isAtLimit;
    //WAS >0 below, changed since now encoder is going negative when extending (0 at stow)
    if ( (tilt.getTiltSpeed() < 0)  && (tilt.isTExtLimit() || tilt.isFullyExtended()) ) {  
      isAtLimit = true;
    } 
    //was < 0 below, changed since now encoder going positive when retracting
    else if ( (tilt.getTiltSpeed() > 0) && (tilt.isTRetLimit()) ) {
      isAtLimit = true; 
    }
    else isAtLimit = false;
   // SmartDashboard.putBoolean("Tilt isFinished condition is: ", isAtLimit);
    return isAtLimit;
  }
}
