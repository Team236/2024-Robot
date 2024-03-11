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
  private double h1 = 44;// inches from ground to center of camera lens
  private double h2 = 57.5; // inches,floor to center of target
  private double a1 = 10*(Math.PI/180); //degrees to rads, camera tilt, up from horizontal
  private double offset = 6.5; //inhces, LL lens to outer edge of bumper
  private double pipeline;
  private double tv, a2, dx, Dx, angleY;

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
        // SmartDashboard.putNumber("LLdx, distance from target:", dx); //test this - use later for cartridge angle equation
         SmartDashboard.putNumber("LLDx, dist woofer to bumper: ", dx-36-offset);
        // SmartDashboard.putNumber("LLty, degrees:", a2);
      } else{
         SmartDashboard.putNumber("No Target", tv);
      }
      Dx = dx - 36 - offset;
      
      //All desiredRevs changed from pos to negative, since tilt motor not inverted
      //So encoder rotations are negative when extending, positive when retracting
      if (Dx < 6) { 
      desiredRevs = -17;  //TODO get actual desiredRevs numbers
    } else if  ((Dx >= 6) && (Dx < 12))  {
      desiredRevs = -23.6;
    } else if  ((Dx >= 12) && (Dx < 18))  {
      desiredRevs = -27.8;
    } else if  ((Dx >= 18) && (Dx < 24))  {
      desiredRevs = -30;
    } else if  ((Dx >= 24) && (Dx < 30))  {
      desiredRevs = -32.5;
    } else if  ((Dx >= 30) && (Dx < 36))  {
      desiredRevs = -36.85;
    } else if  ((Dx >= 36) && (Dx < 39))  {
      desiredRevs = -38.2;
    } else if  ((Dx >= 39) && (Dx < 43))  {
      desiredRevs = -39.42;
    } else if  ((Dx >= 43) && (Dx < 48))  {
      desiredRevs = -41.26;
    } else if ((Dx >= 48) && (Dx < 52)) {
      desiredRevs = -44;
    } else if ((Dx >= 52) && (Dx < 60)) {
      desiredRevs = -45;
    } else if ((Dx >= 60) && (Dx < 65)) {
      desiredRevs = -46;
    } else if ((Dx >= 65) && (Dx < 70)) {
      desiredRevs = -47;
    } else if ((Dx >= 70) && (Dx < 75)) {
      desiredRevs = -48;
    } else if ((Dx >= 75) && (Dx < 80)) {
      desiredRevs = -49;
    } else  {
      desiredRevs = -50;
    }
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
