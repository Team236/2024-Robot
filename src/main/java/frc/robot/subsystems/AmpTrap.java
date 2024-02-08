// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpTrap extends SubsystemBase {
private CANSparkMax motor;
  /** Creates a new TrapAmp. */
  public AmpTrap() {
    motor = new CANSparkMax(Constants.MotorControllers.ID_AMP_TRAP_SHOOTER, MotorType.kBrushless);
    motor.restoreFactoryDefaults();

    motor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
 
    motor.setInverted(true);
  
  } 

    public void shoot(double speed) {
    motor.set(speed);
  }

  public void stop(){
    motor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
