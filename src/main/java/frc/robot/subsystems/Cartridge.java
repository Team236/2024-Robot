// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cartridge extends SubsystemBase { 
  private CANSparkMax leftShooterMotor, rightShooterMotor, tiltMotor;
  private SparkPIDController leftPIDController, rightPIDController, tiltPIDController;
  private RelativeEncoder leftEncoder, rightEncoder, tiltEncoder;
  private boolean isTExtException, isTRetException;
  private DigitalInput tiltExtLimit, tiltRetLimit;


  /** Creates a new CartridgeShooter. */
  public Cartridge() {
    leftShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_SHOOTER_LEFT, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_SHOOTER_RIGHT, MotorType.kBrushless);
    tiltMotor = new CANSparkMax(Constants.MotorControllers.ID_CARTRIDGE_TILT, MotorType.kBrushless);
  

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    tiltMotor.restoreFactoryDefaults();

//TODO determine which one inverted, if any
    leftShooterMotor.setInverted(true);  //*** 
    rightShooterMotor.setInverted(false); //*** 
    tiltMotor.setInverted(false);


    leftShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    tiltMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    leftPIDController = leftShooterMotor.getPIDController();
    rightPIDController = rightShooterMotor.getPIDController();

    leftEncoder = leftShooterMotor.getEncoder();
    rightEncoder = rightShooterMotor.getEncoder();
    tiltEncoder = tiltMotor.getEncoder();

    
  try {
      tiltExtLimit = new DigitalInput(Constants.CartridgeShooter.DIO_TILT_EXT_LIMIT);
    } catch (Exception e) {
       isTExtException = true;
      SmartDashboard.putBoolean("exception thrown for top limit: ", isTExtException);
    }
    try {
      tiltRetLimit = new DigitalInput(Constants.CartridgeShooter.DIO_TILT_RET_LIMIT);
    } catch (Exception e) {
      isTRetException = true;
      SmartDashboard.putBoolean("exception thrown for bottom limit: ", isTRetException);
    }

  }

  //methods start here
  public void setBothSpeeds(double speed) {
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }

  public void closedRampRate() {
    leftShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE); //time in seconds to go from 0 to full throttle
    rightShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE);
  }

  public void openRampRate() {
    leftShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
    rightShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
  }

  //**** NOTE - ShooterMotor PID is done using SPARKMAX PID, BUT TiltMoto PID is done using WPILIB PID **********
  public void setSetpoint(double speed) {
    leftPIDController.setReference(speed, ControlType.kVelocity);
    rightPIDController.setReference(speed, ControlType.kVelocity); //TODO check to set negatives for motors
  }

  public void setP(double kPLeft, double kPRight) {
    leftPIDController.setP(kPLeft);
    rightPIDController.setP(kPRight);
  }

  public void setI(double kILeft, double kIRight) {
    leftPIDController.setI(kILeft);
    rightPIDController.setI(kIRight);
  }

  public void setD(double kDLeft, double kDRight) {
    leftPIDController.setD(kDLeft);
    rightPIDController.setD(kDRight);
  }

  public void setFF(double kFFLeft, double kFFRight) {
    leftPIDController.setFF(kFFLeft);
    rightPIDController.setFF(kFFRight);
  }

  public void setOutputRange() {
    leftPIDController.setOutputRange(0, Constants.CartridgeShooter.MAX_PID_SPEED);
    rightPIDController.setOutputRange(0, Constants.CartridgeShooter.MAX_PID_SPEED);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public double getLeftEncoder() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }

  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }
  
 //**** NOTE - ShooterMotor PID is done using SPARKMAX PID, BUT TiltMoto PID is done using WPILIB PID **********  
  public double getTiltEncoder() {  //gives encoder reading in Revs
    return tiltEncoder.getPosition();
  }
  
  public void resetTiltEncoder() {
    tiltEncoder.setPosition(0);
  }
 
public void stopTilt() {
  tiltMotor.set(0);
}

public boolean isTExtLimit() {
  if (isTExtException) {
    return true;
  } else {
    return tiltExtLimit.get();
  }
}
public boolean isTRetLimit() {
  if (isTRetException) {
    return true;
  } else {
    return tiltRetLimit.get();
  }
}
public boolean isFullyExtended() {
  boolean aFullExtend;
  if (getTiltEncoder() >= Constants.CartridgeShooter.MAX_TILT_ENC_REVS) {
    aFullExtend = true;
  } else {
    aFullExtend = false;      }
  return aFullExtend;
}

public void setTiltSpeed(double speed) {
  if (speed > 0) {  
     //TODO make sure elevator speed > 0 when going up, and top threshold as logical or below
    if (isTExtLimit() || isFullyExtended()) {
        // if fully extended limit is tripped or cartridge at the maximum desired extension and going out, stop 
        stopTilt();
     }  else {
        // cartridge extending out but fully extended limit is not tripped, go at commanded speed
       tiltMotor.set(speed);
      }
    } else {
      if (isTRetLimit()) {
        // cartridge retracting and retract limit is tripped, stop and zero encoder
        stopTilt();
        resetTiltEncoder();
      } else {
        // cartridge retracting but fully retracted limit is not tripped, go at commanded speed
        tiltMotor.set(speed);
      }
     }
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Motor RPM ", getLeftVelocity());
    
    SmartDashboard.putBoolean("Tilt Extend Limit: ", isTExtLimit());
    SmartDashboard.putBoolean("Tilt Retract Limit: ", isTRetLimit());
    SmartDashboard.putNumber("Tilt Encoder REvolutions: ", getTiltEncoder());
    SmartDashboard.putBoolean("Tilte is fully extended: ", isFullyExtended());
  }
}
