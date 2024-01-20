// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private CANSparkMax leftElevatorMotor, rightElevatorMotor;
  private RelativeEncoder leftElevatorEncoder;
  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isElevTopLimitUnplugged, isElevBotLimitUnplugged;
  /** Creates a new Elevator. */
  public Elevator() {
      leftElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_LEFT, MotorType.kBrushless); 
      rightElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_RIGHT, MotorType.kBrushless); 

    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    leftElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT)
 
   
    leftElevatorMotor.setInverted(false);
    rightElevatorMotor.setInverted(true);//TODO check these
  
    leftElevatorEncoder = leftElevatorMotor.getEncoder(); //will use SparkMax encoder for elevator
    
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
    public void resetArmEncoder() {
      armEncoder.setPosition(0); //SparkMax encoder (left only)
    }
   
    //returns encoder position in REVOLUTIONS 
      public double getArmEncoder() {
      return armEncoder.getPosition(); //SparkMax encoder
    }
  
    public double getArmDistance() {
      return  getArmEncoder() * ArmConstants.armREV_TO_IN;
    } 
  
  
  
   //NO LINEAR RELATION BETWEEN PIVOT ANGLE AND ENCODER _ CANNOT USE THIS METHOD
   //NEED TO RECORD PIVOT ENCODER VALUES AT VARIOUS ANGLES AND USE PID COMMANDS WITH THE DESIRED VALUES PASSED IN
   /*  public double getPivotAngle() {
       //could also use turretEncoder.getDistance() here, since dist per pulse is provided at top of this subystem
      return (getPivotEncoder() + PivotConstants.PIVOT_OFFSET_ANGLE)* PivotConstants.pvtDISTANCE_PER_PULSE;
    } //was told this is sketch, need to fix revtodeg constant (becoming unusable w talon) during bench testing*/
  
    public double getTotalArmLength(){
      return (getArmEncoder() * ArmConstants.armREV_TO_IN + ArmConstants.RETRACTED_ARM_LENGTH);
    }
  
    public void setArmSpeed(double speed) {
      armMotor.set(speed);
    if (speed > 0) {
      if (isAExtLimit()) {
          // arm ext/ret limit is tripped going forward, stop 
          // there is 1 magnetic switch and 2 magnets, tripped forward means fully extended
          armStop();
      // ADD AND TEST THE 3 LINES BELOW AFTER INTIAL TESTS OF MANUAL ARM / LIMIT SWITCHES!!!!!!!!!!
      // } else if (arm.getTotalArmLength() > 
      //   (Constants.ArmConstants.MAST_HEIGHT - Constants.ArmConstants.ARM_FLOOR_STANDOFF) / Math.cos(arm.getPivotAngle)){
      //     armStop(); 
       }  else {
          // mast going up but top limit is not tripped, go at commanded speed
          armMotor.set(speed);
        }
      } else {
        if (isARetLimit()) {
          // arm retracting and limit is tripped, stop and zero encoder
          //this means fully retracted
          armStop();
          resetArmEncoder();
        } else {
          // arm retracting but fully retracted limit is not tripped, go at commanded speed
          armMotor.set(speed);
        }
       }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
