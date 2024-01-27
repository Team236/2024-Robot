// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private CANSparkMax leftElevatorMotor, rightElevatorMotor;
  private RelativeEncoder ElevatorEncoder;
  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isElevTopLimitUnplugged, isElevBotLimitUnplugged;
  /** Creates a new Elevator. */
  public Elevator() {
    leftElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_LEFT, MotorType.kBrushless); 
    rightElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_RIGHT, MotorType.kBrushless); 

    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    leftElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
 
    leftElevatorMotor.setInverted(false);
    rightElevatorMotor.setInverted(true);//TODO check these
  
    //TODO  Determine if want to use left or right encoder here, use the one that is increasing when going up
    ElevatorEncoder = leftElevatorMotor.getEncoder(); //will use SparkMax encoder for elevator
    
    try {
      elevatorTopLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_TOP);
    } catch (Exception e) {
      isElevTopLimitUnplugged = true;
    }
    try {
      elevatorBottomLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_BOTTOM);
    } catch (Exception e) {
      isElevBotLimitUnplugged = true;
    }
  }

    public void closedRampRate() {
      leftElevatorMotor.setClosedLoopRampRate(Constants.Elevator.ELEV_CLOSED_RAMP_RATE);
      rightElevatorMotor.setClosedLoopRampRate(Constants.Elevator.ELEV_CLOSED_RAMP_RATE);
    }
  
    public void openRampRate() {
      leftElevatorMotor.setOpenLoopRampRate(Constants.Elevator.ELEV_OPEN_RAMP_RATE);
      rightElevatorMotor.setOpenLoopRampRate(Constants.Elevator.ELEV_OPEN_RAMP_RATE);
    }

    public void elevatorStop() {
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
    }
  
    public boolean isETopLimit() {
      if (isElevTopLimitUnplugged) {
        return true;
      } else {
        return !elevatorTopLimit.get();
      }
    }
    public boolean isEBotLimit() {
      if (isElevBotLimitUnplugged) {
        return true;
      } else {
        return !elevatorBottomLimit.get();
      }
    }
    
  //TODO change to elevator from here down
    public void resetElevatorEncoder() {
      ElevatorEncoder.setPosition(0); //SparkMax encoder (left only)
    }
   
    //returns encoder position in REVOLUTIONS (number of rotations)
      public double getElevatorEncoder() {
      return ElevatorEncoder.getPosition(); //for a SparkMax encoder
    }
  
    public double getElevatorDistance() {
      return  getElevatorEncoder() * Constants.Elevator.ELEV_REV_TO_IN;
    } 

   public double getElevatorLeftSpeed() {
      return  leftElevatorMotor.get();
    }  

      public double getElevatorRightSpeed() {
      return  rightElevatorMotor.get();
    }  
  
  
    public void setElevSpeed(double speed) {
  
    if (speed > 0) {  
       //TODO make sure elevator speed > 0 when going up, and top threshold as logical or below
      if (isETopLimit()) {
          // if elevator limit is tripped or elevator is near the top limit switch going up, stop 
          elevatorStop();
       }  else {
          // elevator going up but top limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
      } else {
        if (isEBotLimit()) {
          // elevator going down and bottom limit is tripped, stop and zero encoder
          elevatorStop();
          resetElevatorEncoder();
        } else {
          // arm retracting but fully retracted limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
        
       }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator distance", getElevatorEncoder());
    SmartDashboard.putBoolean("Elevator at top? ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom? ", isEBotLimit());
    SmartDashboard.putNumber("Elevator Distance: ", getElevatorEncoder());
    SmartDashboard.putNumber("Elevator left speed: ", getElevatorLeftSpeed());
    SmartDashboard.putNumber("Elevator right speed: ", getElevatorRightSpeed());

  }
}
