// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.math.controller.PIDController;
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

  private double dx; //horizontal distance from AprilTag (target) to LL camera lens
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
    
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
      LimelightHelpers.setLEDMode_ForceOff("limelight");
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
      LimelightHelpers.setPipelineIndex("limelight",(int)pipeline); 
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      //tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      //a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    tv = LimelightHelpers.getTV("limelight");

     if(tv==true){   // this could also be get tagId != 0 
        double[] targetCameraPose = LimelightHelpers.getBotPose_TargetSpace("limelight");
        // assumption x target is distance out the from of robot with Limelighthelper
        dx = targetCameraPose[0];  
      } else{
         SmartDashboard.putBoolean("No Target", tv);
      }
  
      // one alternative is to define the Robot to Camera position
      bumperOffset = dx - 36 - cameraOffset;  //From edge of bumper to woofer 

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

/* OLD CODE WITH Dx
    // should be bumperOffset not dx and Dx
    Dx = dx - 36 - offset;  //edge of bumper to woofer
      if (Dx < 3) { 
      desiredRevs = -19;// -17;  
        } else if  ((Dx >= 3) && (Dx < 6))  {
      desiredRevs = -21;//-23.6;
    } else if  ((Dx >= 6) && (Dx < 12))  {
      desiredRevs = -22.6;// -23.6;
    } else if  ((Dx >= 12) && (Dx < 18))  {
      desiredRevs = -30;//-27.8;
    } else if  ((Dx >= 18) && (Dx < 24))  {
      desiredRevs = -35;//-30;
    } else if  ((Dx >= 24) && (Dx < 30))  {
      desiredRevs = -39;//-32.5;
    } else if  ((Dx >= 30) && (Dx < 36))  {
      desiredRevs = -42.3;// -36.85;
    } else if  ((Dx >= 36) && (Dx < 39))  {
      desiredRevs = -43.6;//-38.2;
    } else if  ((Dx >= 39) && (Dx < 42))  {
      desiredRevs = -45;//-39.42;
    } else if  ((Dx >= 42) && (Dx < 45))  {
      desiredRevs = -46;// -41.26;
    } else if ((Dx >= 44) && (Dx < 48)) {
      desiredRevs = -46.8;//-44;
    } else if ((Dx >= 48) && (Dx < 52)) {
      desiredRevs =  -47.2;//-45;
    } else if ((Dx >= 52) && (Dx < 55)) {
      desiredRevs = -48;
    } else if ((Dx >= 55) && (Dx < 65)) {
      desiredRevs = -48.5;//-47;
    } else if ((Dx >= 65) && (Dx < 70)) {
      desiredRevs = -49;//-48;
    } else if ((Dx >= 70) && (Dx < 75)) {
      desiredRevs = -49.5;
    } else  {
      desiredRevs = -50;
    }
    */
    //SmartDashboard.putNumber("Desired Revs", desiredRevs);
    //tilt.setSetpoint(desiredRevs); //old code for when used SparkMax PID
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
