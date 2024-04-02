// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tilt;

public class PidLLTilt extends Command {
  private Tilt tilt;
  private double desiredRevs; //desired encoder revs for the tilt
//Limelight stuff:
    //tV = 1 if there are any targets found, =0 if not
    //ty = vertical offset angle (in radians) from crosshair of LL to target -20.5 to +20.5 degrees
    //h1 = distance from floor to center of Limelight lens
    //h2 = distance from floor to center of target  57.5"?
    //a1 = angle between floor (horizontal) and camera's centerline (camera mount angle, how far rotated from vertical)
    //a2 = getTy (angle between camera's centerline and line extending from center of camera to center of target)
    //dx = horizontal distance from Limelight Lens to target
    //Dx = dx - offset = horizontal distance from robot bumper to target
    //offset = distance from LL lens to outer edge of bumper
    //tan(a1 +a2)  = (h2-h1)/dx;
  private double h1 = 43.5;// inches from ground to center of camera lens
  private double h2 = 57.5;// inches,floor to center of target
  private double a1 = 9.7*(Math.PI/180); //degrees to rads, camera tilt, up from horizontal
  private double pipeline;
  private double tv, a2, angleY;

  private double dx; //horizontal distance from AprilTag (target) to LL camera lens
  private double Dx; // distance from edge of bumper to Woofer
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    angleY = a2 * Math.PI/180;  // a2 in degrees, converted to radians

     if(tv==1){
         dx = (h2 - h1) / Math.tan(a1+angleY);  
        SmartDashboard.putNumber("LLdx, distance from target:", dx); //test this - use later for cartridge angle equation
        //SmartDashboard.putNumber("LLDx, dist woofer to bumper: ", dx-36);
        SmartDashboard.putNumber("LLty, degrees:", a2);
      } else{
         SmartDashboard.putNumber("No Target", tv);
      }
  
      Dx = dx - 36 - cameraOffset;  //From edge of bumper to woofer

    //All desiredRevs changed from pos to negative, since tilt motor not inverted
    //So encoder rotations are negative when extending, positive when retracting

 if (dx < 45.9) {   //old Dx < 3
      desiredRevs = -19;// -17;  //TODO get actual desiredRevs numbers
        } else if  ((dx >= 45.9) && (dx < 50))  {  //old Dx between 3 and 6
      desiredRevs = -22; //-21;
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
    } else if ((dx >= 82.9) && (dx < 85.8)) {  //old Dx between 48 and 52 
      desiredRevs =  -47.2;
    } else if ((dx >= 85.8) && (dx < 88)) { //Old Dx between 52 and 55
      desiredRevs = -48.5;  
        } else if  ((dx >= 88) && (dx < 90))  { //old Dx 55-57
      desiredRevs = -49.8;//
    } else if  ((dx >= 90) && (dx < 92))  { //old Dx between 57-62
      desiredRevs = -50;// 
    } else if  ((dx >= 92) && (dx < 96))  { //old Dx between 62 to 64
      desiredRevs = -50.8;
    } else if  ((dx >= 96) && (dx < 100))  { //old Dx between 64 and 67
      desiredRevs = -51.9;
    } else if  ((dx >= 100) && (dx < 105))  { //old Dx between 67 and 69  //110
      desiredRevs = -52.9;
  //FROM HERE DOWN WERE DONE BY ESTIMATING, AND THEY SEEM TO WORK:
          } else if  ((dx >= 105) && (dx <110))  { //old Dx between 70-72
      desiredRevs = - 54;// 
    } else if  ((dx >= 110) && (dx < 114))  { //old Dx between 72 to 75
      desiredRevs = -55;
    } else if  ((dx >= 114) && (dx < 119))  { //old Dx between 75 and 78
      desiredRevs = -56;
    } else if  ((dx >= 119) && (dx < 122))  { //old Dx between 78 and 81
      desiredRevs = -57;
  //ADDED ON 4/2,  AFTER 4/1/24 PRACTICE - NOT YET TESTED:
    } else if  ((dx >= 122) && (dx < 126))  { //old Dx between 81 and 84
      desiredRevs = -58;
    } else if  ((dx >= 126) && (dx < 130))  { //old Dx between 84 and 88
      desiredRevs = -59;
    } else  {
      desiredRevs = -60; 
    }

/* OLD CODE WITH Dx
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
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
