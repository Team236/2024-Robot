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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cartridge extends SubsystemBase { 
  private CANSparkMax leftShooterMotor, rightShooterMotor;
  private SparkPIDController leftPIDController, rightPIDController;
  private RelativeEncoder leftCartEncoder, rightCartEncoder;

  /** Creates a new CartridgeShooter. */
  //USING WPILib PID, not SparkMax PID.  SparkMax PID is not consistent - intermittent issues
  public Cartridge() {
    leftShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_CARTRIDGE_LEFT, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_CARTRIDGE_RIGHT, MotorType.kBrushless);
  
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    leftShooterMotor.setInverted(true);  
    rightShooterMotor.setInverted(false); 

    leftPIDController = leftShooterMotor.getPIDController();
    rightPIDController = rightShooterMotor.getPIDController();

    //TODO: determine which encoder is increasing when going up - 
    //CartEncoder = leftShooterMotor.getEncoder();
    rightCartEncoder = rightShooterMotor.getEncoder();
    leftCartEncoder = leftShooterMotor.getEncoder();
  }

  //methods start here
  public void setBothSpeeds(double speed) {
    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
  }
/* 
  public void closedRampRate() {
    leftShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE); //time in seconds to go from 0 to full throttle
    rightShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE);
  }

  public void openRampRate() {
    leftShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
    rightShooterMotor.setClosedLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
  }
*/
  //**** NOTE - PID is done using SPARKMAX PID**********
  public void setSetpoint (double speedL, double speedR) {
  //public void setSetpoint(double speed) {
    leftPIDController.setReference(speedL, ControlType.kVelocity);
    rightPIDController.setReference(speedR, ControlType.kVelocity); 
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
    leftCartEncoder.setPosition(0);
    rightCartEncoder.setPosition(0);
  }

  public double getLeftCartEncoder() {
    return leftCartEncoder.getPosition();
  }
  public double getRightCartEncoder() {
    return rightCartEncoder.getPosition();
  }


  public double getLeftVelocity() {
    return leftCartEncoder.getVelocity();
  }

   public double getRightVelocity() {
    return rightCartEncoder.getVelocity();
  }
   
  //NOT SURE IF THIS METHOD WORKS. WAS GOING TO USE IT IN ROBOT.JAVA, AUTO PERIODIC, 
  //TO KEEP CART MOTORS RUNNING IN AUTO
    public void setRPM_PID(double desiredRPMLeft, double desiredRPMRight) {
      setP(Constants.CartridgeShooter.kPLeft, Constants.CartridgeShooter.kPRight);
      setI(Constants.CartridgeShooter.kILeft, Constants.CartridgeShooter.kIRight);
      setD(Constants.CartridgeShooter.kDLeft, Constants.CartridgeShooter.kDRight);
      setFF(Constants.CartridgeShooter.kFFLeft, Constants.CartridgeShooter.kFFRight);
      setOutputRange();
      setSetpoint(desiredRPMLeft, desiredRPMRight);
    }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Cart Motor RPM ", getLeftVelocity());
    SmartDashboard.putNumber("Right Cart Motor RPM ", getRightVelocity());
   // SmartDashboard.putNumber("Left Cart encoder: ", getLeftCartEncoder());
   // SmartDashboard.putNumber("Right Cart encoder: ", getRightCartEncoder());
  }
}
