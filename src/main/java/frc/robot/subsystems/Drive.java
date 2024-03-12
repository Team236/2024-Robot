// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Thrustmaster;

public class Drive extends SubsystemBase {
  public CANSparkMax leftFront, leftRear, rightFront, rightRear;
  public DifferentialDrive diffDrive;
  private DoubleSolenoid transmission;

  //these are external encoders not SparkMAX
  private Encoder leftEncoder, rightEncoder;

  /** Creates a new Drive. */
  public Drive() {
    leftFront = new CANSparkMax(Constants.MotorControllers.ID_LEFT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.MotorControllers.ID_LEFT_REAR, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.MotorControllers.ID_RIGHT_FRONT, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.MotorControllers.ID_RIGHT_REAR, MotorType.kBrushless);

    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();

    leftFront.setInverted(false);
    rightFront.setInverted(true);
 
    leftFront.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightFront.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    leftRear.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightRear.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    //creates a new diffdrive
    diffDrive = new DifferentialDrive(leftFront, rightFront); 
    diffDrive.setSafetyEnabled(false);
    diffDrive.setDeadband(Constants.DriveConstants.DEADBAND);

    //external encoders
    leftEncoder = new Encoder(Constants.DriveConstants.DIO_LDRIVE_ENC_A, Constants.DriveConstants.DIO_LDRIVE_ENC_B); 
    rightEncoder = new Encoder(Constants.DriveConstants.DIO_RDRIVE_ENC_A, Constants.DriveConstants.DIO_RDRIVE_ENC_B); 

    rightEncoder.setDistancePerPulse(Constants.DriveConstants.DISTANCE_PER_PULSE_K);
    leftEncoder.setDistancePerPulse(Constants.DriveConstants.DISTANCE_PER_PULSE_K);

    //pneumatic double solenoid
    transmission = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.DriveConstants.SOL_LOW_GEAR, Constants.DriveConstants.SOL_HIGH_GEAR);
  }

  //methods start here
public void setGearHigh(){
  transmission.set(Value.kReverse);
}

public void setGearLow(){
  transmission.set(Value.kForward);
}

public boolean isInLowGear(){
  return transmission.get() == Value.kForward;
}

public boolean isInHighGear(){
  return transmission.get() == Value.kReverse;
}
//CAREFUL!! MUST USE RAMPRATE EXACTLY IN ORDER AS SHOWN IN ARCADE DRIVE COMMAND
//OR FOLLOWER STOPS WORKING!!!!!
public void openRampRate() {
  leftFront.setOpenLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
  rightFront.setOpenLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
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

public double getLeftEncoder() {
  //return leftEncoder.get
  return leftEncoder.getRaw();
}

public double getRightEncoder() {
    return rightEncoder.getRaw();
  }
  public double getLeftDistance() {
    return getLeftEncoder() * DriveConstants.DISTANCE_PER_PULSE_K;
    // distance per pulse * encoder reading = inches
  }
  public double getRightDistance() {
    //return rightEncoder.getDistance();
    return getRightEncoder() * DriveConstants.DISTANCE_PER_PULSE_K;
  }
  public double getAvgDistance() {
    return (getLeftDistance() + getRightDistance())/2 ;
  }
  public void resetLeftEncoder() {
    leftEncoder.reset();
  }
  public void resetRightEncoder() {
    rightEncoder.reset();
    }


public void stop() {
  leftFront.set(0);
  rightFront.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.getBoolean("In High Gear?", isInHighGear());
    //SmartDashboard.putNumber("left Encoder Ticks", getLeftEncoder());
    //SmartDashboard.putNumber("Right Encoder Ticks", getRightEncoder());
    SmartDashboard.putNumber("Left Dist: ", getLeftDistance());
    SmartDashboard.putNumber("Right Dist: ", getRightDistance());
  }
}
