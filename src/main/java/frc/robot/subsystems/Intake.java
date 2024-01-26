// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeLeft, intakeRight;
  public static Counter counter;
  private boolean isCounterUnplugged = false;


//TODO add optical sensor/counter

  /** Creates a new Intake. */
  public Intake() {
    intakeLeft = new CANSparkMax(Constants.MotorControllers.ID_INTAKE_LEFT, MotorType.kBrushless);
    intakeRight = new CANSparkMax(Constants.MotorControllers.ID_INTAKE_RIGHT, MotorType.kBrushless);

    intakeLeft.restoreFactoryDefaults();
    intakeRight.restoreFactoryDefaults();

    intakeLeft.setInverted(false); //TODO determine inverted motors
    intakeRight.setInverted(true);

    intakeLeft.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    intakeRight.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    try {
      counter = new Counter();
      counter.setUpSource(Constants.Intake.DIO_INTAKE_COUNTER);
      counter.reset();
    } catch (Exception e) {
      isCounterUnplugged = true;
    }

  }

  //Methods start here
  public void openRampRate() {
    intakeLeft.setOpenLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
    intakeRight.setOpenLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
  }

  public void closedRampRate() {
    intakeLeft.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE);
    intakeRight.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE);
  }

  public void intakeStop() {
    intakeLeft.set(0);
    intakeRight.set(0);
  }

  public void setIntakeSpeed(double speed) {
    intakeLeft.set(speed);
    intakeRight.set(speed);
  }

  public int getIntakeCount() {
    int count = 0;

    if (isCounterUnplugged) {
      SmartDashboard.putBoolean("Intake counter unplugged:", isCounterUnplugged);
   
    } else {
      count =  counter.get();
    }
    return count;
  }

  public void resetCounter() {
    counter.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake counter unplugged: ", isCounterUnplugged);
    
    SmartDashboard.putNumber("Intake periodic count is:", getIntakeCount());
  }
}
