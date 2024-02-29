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
private CANSparkMax intake;
public static Counter counter;
private static boolean isCounterUnplugged = false;

  /** Creates a new Intake. */
  public Intake() {
    intake= new CANSparkMax(Constants.MotorControllers.ID_INTAKE, MotorType.kBrushless);

    intake.restoreFactoryDefaults();

    intake.setInverted(false); 

    intake.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    try {
      counter = new Counter();
      counter.setUpSource(Constants.Intake.DIO_COUNTER);
      counter.reset();
    } 
    catch (Exception e) {
      isCounterUnplugged = true;
    }
 
    SmartDashboard.putBoolean("is counter unplugged:", isCounterUnplugged);
    counter.reset(); //sets counter to zero
  }

public void resetCount() {
  counter.reset();
}

 //Methods start here
  public void openRampRate() {
    intake.setOpenLoopRampRate(Constants.MotorControllers.OPEN_RAMP_RATE);
  }

  public void intakeStop() {
    intake.set(0);
  }

  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }
 /* 
public int getCount() {
  return counter.get();
}
*/
//TWO STATIC METHODS BELOW - TO KEEP COUNT ACROSS VARIOUS SUBSYSTEMS
  public static int getIntakeCount() {
    int count;
    if (isCounterUnplugged) {
      count = 0;
      SmartDashboard.putBoolean("Intake counter unplugged:", isCounterUnplugged);
    } else {
      count =  counter.get();
    }
    return count;
  }

  public static void resetCounter() {
    counter.reset();
  }

  public boolean isIntakeSpinning() {
    boolean spin;
    if (Math.abs(intake.get()) >0.1) {
      spin = true;
    }
    else {
      spin = false;
    }
    return spin;
  }

    /* 
  public void closedRampRate() {
    intake.setClosedLoopRampRate(Constants.MotorControllers.CLOSED_RAMP_RATE);
  }
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putNumber("Intake periodic count is:", getIntakeCount());
   SmartDashboard.putBoolean("HasNote: ", counter.get()>0);
   //SmartDashboard.putBoolean("is intake spinning ", isIntakeSpinning());
}

}