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
  private RelativeEncoder leftEncoder, rightEncoder;

  /** Creates a new CartridgeShooter. */
  //TODO:  CHECK IF CARTRIDGE POSITION HOLDS WHEN PIDCartridgeTilt ends
  public Cartridge() {
    leftShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_CARTRIDGE_LEFT, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_CARTRIDGE_RIGHT, MotorType.kBrushless);
  
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterMotor.setInverted(true);  //*** 
    rightShooterMotor.setInverted(false); //*** 

    leftShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    leftPIDController = leftShooterMotor.getPIDController();
    rightPIDController = rightShooterMotor.getPIDController();

    leftEncoder = leftShooterMotor.getEncoder();
    rightEncoder = rightShooterMotor.getEncoder();
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
  //**** NOTE - ShooterMotor PID is done using SPARKMAX PID, BUT TiltMoto PID is done using WPILIB PID **********
  public void setSetpoint(double speed) {
    leftPIDController.setReference(speed, ControlType.kVelocity);
    rightPIDController.setReference(speed, ControlType.kVelocity); 
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Motor RPM ", getLeftVelocity());
  }
}
