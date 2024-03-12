// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tilt extends SubsystemBase {
  private CANSparkMax tiltMotor;
  private RelativeEncoder tiltEncoder;
  //SWITCHED TO WPILib PID, not SPARKMAX PID
  //private SparkPIDController tiltPIDController;
  private boolean isTExtException, isTRetException;
  private DigitalInput tiltExtLimit, tiltRetLimit;

  /** Creates a new Tilt. */
  public Tilt() {
    tiltMotor = new CANSparkMax(Constants.MotorControllers.ID_CARTRIDGE_TILT, MotorType.kBrushless);

    tiltMotor.restoreFactoryDefaults();
    tiltMotor.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    tiltMotor.setInverted(false);//WAS TRUE - NOW USE NEGATIVE ENC VALUES TO TILT
    tiltEncoder = tiltMotor.getEncoder();
   // tiltPIDController = tiltMotor.getPIDController();

      try {
      tiltExtLimit = new DigitalInput(Constants.Tilt.DIO_TILT_EXT_LIMIT);
    } catch (Exception e) {
       isTExtException = true;
      SmartDashboard.putBoolean("exception thrown for Tilt top limit: ", isTExtException);
    }
    try {
      tiltRetLimit = new DigitalInput(Constants.Tilt.DIO_TILT_RET_LIMIT);
    } catch (Exception e) {
      isTRetException = true;
      SmartDashboard.putBoolean("exception thrown for Tilt bottom limit: ", isTRetException);
    }
  }
//MEHTODS START HERE
  //**** NOTE - ShooterMotor PID is done using SPARKMAX PID, AND TILT MOTOR AS WELL ****** 
  public double getTiltEncoder() {  //gives encoder reading in Revs
    return tiltEncoder.getPosition();
  }
  
  public void resetTiltEncoder() {
    tiltEncoder.setPosition(0);
  }
 
public void stopTilt() {
  tiltMotor.set(0);
}

public boolean isTExtLimit() {
  if (isTExtException) {
    return true;
  } else {
    return tiltExtLimit.get();
  }
}

public boolean isTRetLimit() {
  if (isTRetException) {
    return true;
  } else {
    return tiltRetLimit.get();
  }
}

public boolean isFullyExtended() {
  boolean aFullExtend;
  if (getTiltEncoder() <= Constants.Tilt.TILT_ENC_REVS_MAX) {  //WAS >=
    aFullExtend = true;
  } else {
    aFullExtend = false;      }
  return aFullExtend;
}

public void setTiltSpeed(double speed) {
  if (speed <= 0) {  //was >0 but changed since going negative when extending now
     //TODO make sure elevator speed > 0 when going up
    if (isTExtLimit() || isFullyExtended()) {
        // if fully extended limit is tripped or cartridge at the maximum desired extension and going out, stop 
        stopTilt();
     }  else {
        // cartridge extending out but fully extended limit is not tripped, go at commanded speed
       tiltMotor.set(speed);
      }
 } 
 else {
      if (isTRetLimit()) {
        // cartridge retracting and retract limit is tripped, stop and zero encoder
        stopTilt();
        resetTiltEncoder();
      } else {
        // cartridge retracting but fully retracted limit is not tripped, go at commanded speed
        tiltMotor.set(speed);
      }
     }
}

public double getTiltSpeed() {
  return tiltMotor.get();
}

//!!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID 
//**** CHANGED BACK TO USING WPILib PID ****
//**** due to spurious encoder polarity changes when run multiple autos in a row ****
/*
public void setSetpoint(double encoderRevs) {
  tiltPIDController.setReference(encoderRevs, ControlType.kPosition);
}

public void setP(double kP) {
  tiltPIDController.setP(kP);
}

public void setI(double kI) {
  tiltPIDController.setI(kI);
}

public void setD(double kD) {
  tiltPIDController.setD(kD);
}

public void setFF(double kFF) {
  tiltPIDController.setFF(kFF);
}
*/
    @Override
  public void periodic() {
    SmartDashboard.putBoolean("Tilt Extend Limit: ", isTExtLimit());
    SmartDashboard.putBoolean("Tilt Retract Limit: ", isTRetLimit());
    SmartDashboard.putNumber("Tilt Encoder Revolutions: ", getTiltEncoder());
    SmartDashboard.putBoolean("Tilt is fully extended: ", isFullyExtended());
  }

  }


