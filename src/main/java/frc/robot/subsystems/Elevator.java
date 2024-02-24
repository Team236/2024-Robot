// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  //USING WPILib PID, not SparkMax PID.  SparkMax PID is not consistent - intermittent issues
  private CANSparkMax leftElevatorMotor, rightElevatorMotor;
  //private SparkPIDController leftPIDController, rightPIDController;
  public RelativeEncoder leftElevEncoder, rightElevEncoder;
  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isTException, isBException;
  private DoubleSolenoid brake;
  /** Creates a new Elevator. */
    //TODO:  CHECK IF ELEVATOR POSITION HOLDS WHEN PID up ends
  public Elevator() {
    leftElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_LEFT, MotorType.kBrushless); 
    rightElevatorMotor = new CANSparkMax(Constants.MotorControllers.ID_ELEVATOR_RIGHT, MotorType.kBrushless); 

    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    leftElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightElevatorMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
 
    leftElevatorMotor.setInverted(true);
    rightElevatorMotor.setInverted(false);

    //leftElevatorMotor.follow(rightElevatorMotor);
    //leftPIDController = leftElevatorMotor.getPIDController();
    // rightPIDController = rightElevatorMotor.getPIDController();
  
    leftElevEncoder = leftElevatorMotor.getEncoder(); //will use SparkMax encoder for elevator
    rightElevEncoder = rightElevatorMotor.getEncoder();

    try {
      elevatorTopLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_TOP);
    } catch (Exception e) {
      isTException = true;
      SmartDashboard.putBoolean("exception thrown for Elev top limit: ", isTException);
    }
    try {
      elevatorBottomLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_BOTTOM);
    } catch (Exception e) {
      isBException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isBException);
    }

    //pneumatic double solenoid
    brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Elevator.SOL_BRAKE_ON, Constants.Elevator.SOL_BRAKE_OFF);
  }

//methods start here:
  public void engageBrake(){
   brake.set(Value.kReverse);
}

public void removeBrake(){
   brake.set(Value.kForward);
}

public boolean isBrake(){
  return brake.get() == Value.kReverse;
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
   
    public void resetElevatorEncoders() {
      leftElevEncoder.setPosition(0); //SparkMax encoder (left only)
      rightElevEncoder.setPosition(0);
    }
   
    //returns encoder position in REVOLUTIONS (number of rotations)
      public double getElevLeftEncoder() {
      return leftElevEncoder.getPosition(); //for a SparkMax encoder
    }
      public double getElevRightEncoder() {
      return rightElevEncoder.getPosition(); //for a SparkMax encoder
    }
    //reads elevator distance travelled in inches 
    //!!!!!!Use encoder that is increasing when going up to evaluate distance travelled:
    public double getElevatorHeight() {
      return  getElevLeftEncoder() * Constants.Elevator.ELEV_REV_TO_IN;
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
      if (isETopLimit() || isTop()) {
          // if elevator limit is tripped or elevator is near the top limit switch going up, stop 
          stopElevator();
       }  else {
          // elevator going up but top limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
      } 
      else {
        if (isEBotLimit()) {
          //elevator going down and is at the bottom,stop and zero encoder
          stopElevator();
          resetElevatorEncoders();
        } else {
        // elevator going down but not at the bottom, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
       }
         } 

//!!!! SPARKMAX PID STUFF - DECIDED NOT TO USE SPARKMAX PID (using WPILib PID)
//ISSUE WITH SPARKMAX LIKELY DUE TO NOT RESETTING BOTH LEFT AND RIGHT ENCODERS WHEN BOTTOM LIMIT HIT
/* public void setSetpoint(double encoderRevs) {
  leftPIDController.setReference(encoderRevs, ControlType.kPosition);
  rightPIDController.setReference(encoderRevs, ControlType.kPosition);
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
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("BrakeEngaged?: ", isBrake());
    SmartDashboard.putNumber("Elevator height: ", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator at top? ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom? ", isEBotLimit());
    //SmartDashboard.putNumber("Left elevator encoder", getElevLeftEncoder()); 
    //SmartDashboard.putNumber("Right elevator encoder", getElevRightEncoder());

   // SmartDashboard.putNumber("Elevator left speed: ", getElevatorLeftSpeed());
   // SmartDashboard.putNumber("Elevator right speed: ", getElevatorRightSpeed());
  }
}
