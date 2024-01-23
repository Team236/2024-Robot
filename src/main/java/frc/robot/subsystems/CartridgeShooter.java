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

public class CartridgeShooter extends SubsystemBase {
  private DoubleSolenoid solenoidLongRange;
  private DoubleSolenoid solenoidShortRange;
  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;


  /** Creates a new CartridgeShooter. */
  public CartridgeShooter() {
    solenoidLongRange = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CartridgeShooter.SOL_LONG_RANGE_FORWARD, Constants.CartridgeShooter.SOL_LONG_RANGE_REVERSE);
    solenoidShortRange = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CartridgeShooter.SOL_SHORT_RANGE_FORWARD, Constants.CartridgeShooter.SOL_SHORT_RANGE_REVERSE);
    leftShooterMotor = new CANSparkMax(Constants.CartridgeShooter.ID_LEFT_SHOOTER, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.CartridgeShooter.ID_RIGHT_SHOOTER, MotorType.kBrushless);

    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

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

  public void setPositionHigh(boolean isLongRange) {
    if(isLongRange) {
       solenoidLongRange.set(Value.kReverse); //TODO assume kreverse is putting cartridge in high position
    } else {
       solenoidShortRange.set(Value.kReverse);
    }
  }
// we can try combining high and low
public void setPositionLow(boolean isLongRange) {
    if(isLongRange) {
       solenoidLongRange.set(Value.kForward); //TODO assume kforward is putting cartridge in low position
    } else {
       solenoidShortRange.set(Value.kForward); 
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
