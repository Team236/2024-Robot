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

public class Elevator extends SubsystemBase {
  private CANSparkMax leftElevatorMotor, rightElevatorMotor;
  private SparkPIDController leftPIDController, rightPIDController; //!!!!
  private RelativeEncoder elevatorEncoder;
  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isTException, isBException;
  /** Creates a new Elevator. */
  public Elevator() {
    leftElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_LEFT, MotorType.kBrushless); 
    rightElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_RIGHT, MotorType.kBrushless); 

    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    leftElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
 
    leftElevatorMotor.setInverted(true);
    rightElevatorMotor.setInverted(false);//TODO check these

    leftPIDController = leftElevatorMotor.getPIDController();
    rightPIDController = rightElevatorMotor.getPIDController();
  
    //TODO  Determine if want to use left or right encoder here, use the one that is increasing when going up
    elevatorEncoder = leftElevatorMotor.getEncoder(); //will use SparkMax encoder for elevator
    
    try {
      elevatorTopLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_TOP);
    } catch (Exception e) {
      isTException = true;
      SmartDashboard.putBoolean("exception thrown for top limit: ", isTException);
    }
    try {
      elevatorBottomLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_BOTTOM);
    } catch (Exception e) {
      isBException = true;
      SmartDashboard.putBoolean("exception thrown for bottom limit: ", isBException);
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

    public void stopElevator() {
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
    }
  
    public boolean isETopLimit() {
      if (isTException) {
        return true;
      } else {
        return elevatorTopLimit.get();
      }
    }
    public boolean isEBotLimit() {
      if (isBException) {
        return true;
      } else {
        return elevatorBottomLimit.get();
      }
    }

    public void resetElevatorEncoder() {
      elevatorEncoder.setPosition(0); //SparkMax encoder (left only)
    }
   
    //returns encoder position in REVOLUTIONS (number of rotations)
      public double getElevatorEncoder() {
      return elevatorEncoder.getPosition(); //for a SparkMax encoder
    }

    //reads elevator distance travelled in inches 
    public double getElevatorHeight() {
      return  getElevatorEncoder() * Constants.Elevator.ELEV_REV_TO_IN;
    } 

    public boolean isTop() {
      boolean eTop;
      if (getElevatorHeight() >= Constants.Elevator.MAX_HEIGHT) {
        eTop = true;
      } else {
        eTop = false;      }
      return eTop;
    }


   public double getElevatorLeftSpeed() {
      return  leftElevatorMotor.get();
    }  

      public double getElevatorRightSpeed() {
      return  rightElevatorMotor.get();
    }  
  
  
    public void setElevSpeed(double speed) {
    if (speed > 0) {  
       //TODO make sure elevator speed > 0 when going up
      if (isETopLimit() || isTop()) {
          // if elevator limit is tripped or elevator is near the top limit switch going up, stop 
          stopElevator();
       }  else {
          // elevator going up but top limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
      } else {
        if (isEBotLimit()) {
          // elevator going down and bottom limit is tripped, stop and zero encoder
          stopElevator();
          resetElevatorEncoder();
        } else {
          // arm retracting but fully retracted limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
       }
         }

//!!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID 
//**** NOTE - This PID is done using SPARKMAX PID, BUT DRIVE PID is with WPILIB PID *******
 public void setSetpoint(double speed) {
  leftPIDController.setReference(speed, ControlType.kPosition);
  rightPIDController.setReference(speed, ControlType.kPosition); //TODO check to set negatives for motors
}

public void setP(double kP) {
  leftPIDController.setP(kP);
  rightPIDController.setP(kP);
}

public void setI(double kI) {
  leftPIDController.setI(kI);
  rightPIDController.setI(kI);
}

public void setD(double kD) {
  leftPIDController.setD(kD);
  rightPIDController.setD(kD);
}

public void setFF(double kFF) {
  leftPIDController.setFF(kFF);
  rightPIDController.setFF(kFF);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator height", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator at top? ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom? ", isEBotLimit());
  }
}
