// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Thrustmaster;

public class Drive extends SubsystemBase {
  public CANSparkMax leftFront, leftRear, rightFront, rightRear;
  public DifferentialDrive diffDrive;

  //these are external encoders not SparkMAX
  private Encoder leftEncoder, rightEncoder;

  public XboxController driverController;//use for XBox controller
  public Joystick leftStick, rightStick;//use for Thrustmaster joysticks

  /** Creates a new Drive. */
  public Drive() {
    driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);

    leftStick = new Joystick(Constants.Controller.USB_LEFT_JOYSTICK);
    rightStick = new Joystick(Constants.Controller.USB_RIGHT_JOYSTICK);

    diffDrive = new DifferentialDrive(leftFront, rightFront); //TODO rears are being commanded?

    leftFront = new CANSparkMax(Constants.MotorControllers.ID_LEFT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.MotorControllers.ID_LEFT_REAR, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.MotorControllers.ID_RIGHT_FRONT, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.MotorControllers.ID_RIGHT_REAR, MotorType.kBrushless);

    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();

    leftFront.setInverted(false);
    rightFront.setInverted(true); //determine via bench testing
    
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftFront.setSmartCurrentLimit(40);
    rightFront.setSmartCurrentLimit(40);
    leftRear.setSmartCurrentLimit(40);
    rightRear.setSmartCurrentLimit(40);

    leftEncoder = new Encoder(Constants.DriveConstants.DIO_LDRIVE_ENC_A, Constants.DriveConstants.DIO_LDRIVE_ENC_B); 
    rightEncoder = new Encoder(Constants.DriveConstants.DIO_RDRIVE_ENC_A, Constants.DriveConstants.DIO_RDRIVE_ENC_B); 

    rightEncoder.setDistancePerPulse(Constants.DriveConstants.DISTANCE_PER_PULSE_K);
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.DISTANCE_PER_PULSE_K);
  }

  //methods start here
public void closedRampRate() {
  leftFront.setClosedLoopRampRate(0.08);
  rightFront.setClosedLoopRampRate(0.08);
}
public void openRampRate() {
  leftFront.setClosedLoopRampRate(0.08);
  rightFront.setClosedLoopRampRate(0.08);
}

public void setLeftSpeed(double speed) {
  leftFront.set(speed);
}

public void setRightSpeed(double speed) {
  rightFront.set(speed);
}

public void setBothSpeeds(double speed) {
  leftFront.set(speed);
  rightFront.set(speed);
}

public void setTurnCWSpeeds(double speed) {
  leftFront.set(speed);
  rightFront.set(-speed);
}

public void setTurnCCWSpeeds(double speed) {
  leftFront.set(-speed);
  rightFront.set(speed);
}

public double getLeftSpeed(){
  //return leftEncoder.getVelocity(); //use for internal SparkMax encoder?
  
  //getRate units are distance per second, as scaled by the value of DistancePerPulse
  return leftEncoder.getRate(); //use for external drive encoders
}

public double getRightSpeed(){
  //return leftEncoder.getVelocity(); //use for internal SparkMax encoder?
  return rightEncoder.getRate(); //use for external drive encoders
}

//TODO - add in methods getLeftEncoder, getRightEncoder, getLeftDistance,
// getRightDistance, getAvgDistance, resetLeftEncoder, resetRightEncoder

public void stop() {
  leftFront.set(0);
  rightFront.set(0);
}

public boolean inYDeadzone () {
  return (Math.abs(driverController.getLeftY()) <= Constants.DriveConstants.RIGHT_DEADZONE);
}

public boolean inXDeadzone () {
  return (Math.abs(driverController.getRightX()) <= Constants.DriveConstants.LEFT_DEADZONE);
}

public void driveArcade(double leftYY, double rightXX) {
  //arcadeDrive(speed, rotation)
  //TODO  - replace last line in this method with the 2 lines below and see effect
  //SlewRateLimiter filter = new SlewRateLimiter(0.5);
  //diffDrive.arcadeDrive(filter.calculate(leftYY), rightXX);
  diffDrive.arcadeDrive (leftYY, rightXX);
}
public void ArcadeWithDeadzone(){
  //This method adds deadzones into the driving, using XBox controller for driving
  //arcadeDrive(speed, rotation) - speed is Y-axis of left stick, rotation is x-axis of right stick
  //xspeed is getLeftY, zrotation is getRightX, for arcade drive with 2 sticks
  if(inYDeadzone() && inXDeadzone()) {
    driveArcade(0,0);
  } else if(!inYDeadzone() && !inXDeadzone()) { 
    driveArcade(-driverController.getLeftY(), -driverController.getRightX());
  } else if(inYDeadzone() && !inXDeadzone()) {
    driveArcade( 0, -driverController.getRightX());
  } else if(!inYDeadzone() && inXDeadzone()) {
    driveArcade(-driverController.getLeftY(), 0);
}
}

public void driveTank(double leftSpeed, double rightSpeed){
  //tankDrive(leftSpeed, rightSpeed))
  //TODO  - replace last line in this method with the 2 lines below and see effect
  //SlewRateLimiter filter = new SlewRateLimiter(0.5);
  //diffDrive.tankDrive(filter.calculate(leftSpeed), rightSpeed);
  diffDrive.tankDrive(leftSpeed, rightSpeed);
}
public void TankWithDeadzone(){
   //This method adds deadzones into the driving, using XBox controller for driving
   //tankDrive(leftSpeed, rightSpeed))
  if(inYDeadzone() && inXDeadzone()) {
    driveTank(0, 0);
  } else if(!inYDeadzone() && !inXDeadzone()) { 
    driveTank(-driverController.getLeftY(), -driverController.getRightY());
  } else if(inYDeadzone() && !inXDeadzone()) {
    driveTank( 0, -driverController.getRightY());
  } else if(!inYDeadzone() && inXDeadzone()) {
    driveTank(-driverController.getLeftY(), 0);
}
}

public void ArcadeWithSticks(){
  //This method adds deadzones into the driving, using 2 joysticks for driving
  //arcadeDrive(speed, rotation) - speed is Y-axis of left stick, rotation is x-axis of right stick
  //xspeed is getLeftY, zrotation is getRightX, for arcade drive with 2 sticks
  if(inYDeadzone() && inXDeadzone()) {
    driveArcade(0,0);
  } else if(!inYDeadzone() && !inXDeadzone()) { 
    driveArcade(-leftStick.getY(), -rightStick.getX());
  } else if(inYDeadzone() && !inXDeadzone()) {
    driveArcade( 0, -rightStick.getX());
  } else if(!inYDeadzone() && inXDeadzone()) {
    driveArcade(-leftStick.getY(), 0);
}
}

public void TankWithSticks(){
   //This method adds deadzones into the driving, using XBox controller for driving
   //tankDrive(leftSpeed, rightSpeed))
  if(inYDeadzone() && inXDeadzone()) {
    driveTank(0, 0);
  } else if(!inYDeadzone() && !inXDeadzone()) { 
    driveTank(-leftStick.getY(), -rightStick.getY());
  } else if(inYDeadzone() && !inXDeadzone()) {
    driveTank( 0, -rightStick.getY());
  } else if(!inYDeadzone() && inXDeadzone()) {
    driveTank(-leftStick.getY(), 0);
}
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
