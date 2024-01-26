// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cartridge extends SubsystemBase {
  private DoubleSolenoid solenoid1;//*** 
  private DoubleSolenoid solenoid2; //*** 
  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;


  /** Creates a new CartridgeShooter. */
  public Cartridge() {
    solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CartridgeShooter.SOL_CARTRIDGE_1_FWD, Constants.CartridgeShooter.SOL_CARTRIDGE_1_REV);
    solenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CartridgeShooter.SOL_CARTRIDGE_2_FWD, Constants.CartridgeShooter.SOL_CARTRIDGE_2_REV);
    leftShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_SHOOTER_LEFT, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.MotorControllers.ID_SHOOTER_RIGHT, MotorType.kBrushless);

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
//TODO determine which one inverted, if any
    leftShooterMotor.setInverted(true);  //*** 
    rightShooterMotor.setInverted(false); //*** 

    leftShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightShooterMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    
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

  // *** several methods updated, other deleted below
  // *** //set both solenoids Reverse for stowed
  public void cartridgeStowedPosition(){  //*** 
    solenoid1.set(Value.kReverse);
    solenoid2.set(Value.kReverse);
  }
  //*** //set one solenoid Forward for Woofer shot
  public void wooferShotPosition(){
    solenoid1.set(Value.kForward);
    solenoid2.set(Value.kReverse);
  }
  //*** set two solenoids forward for Podium shot
  public void podiumShotPosition(){
    solenoid1.set(Value.kForward);
    solenoid2.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
