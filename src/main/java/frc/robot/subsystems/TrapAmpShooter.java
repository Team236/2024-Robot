// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapAmpShooter extends SubsystemBase {
private CANSparkMax motor;
  /** Creates a new TrapAmpShooter. */
  public TrapAmpShooter() {
    motor = new CANSparkMax(Constants.MotorControllers.ID_AMP_TRAP_SHOOTER, MotorType.kBrushless);
  } 

  public void shoot() {
    motor.set(Constants.AMP.AMP_TRAP_MOTOR_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
