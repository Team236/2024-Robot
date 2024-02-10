// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class OdometryDrive extends SubsystemBase {

	private CANSparkMax leftFront, leftRear, rightFront, rightRear;
	private DifferentialDrive diffDrive;
	private DoubleSolenoid transmission;

	//these are external encoders not SparkMAX
	private Encoder leftEncoder, rightEncoder;
	
		/** Creates a new OdometryDrive. */				 
	public OdometryDrive() {
	leftFront = new CANSparkMax(MotorControllers.ID_LEFT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(MotorControllers.ID_LEFT_REAR, MotorType.kBrushless);
    rightFront = new CANSparkMax(MotorControllers.ID_RIGHT_FRONT, MotorType.kBrushless);
    rightRear = new CANSparkMax(MotorControllers.ID_RIGHT_REAR, MotorType.kBrushless);

    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();

    leftFront.setInverted(false);
    rightFront.setInverted(true); //determine via bench testing
    
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftFront.setSmartCurrentLimit(MotorControllers.SMART_CURRENT_LIMIT);
    rightFront.setSmartCurrentLimit(MotorControllers.SMART_CURRENT_LIMIT);
    leftRear.setSmartCurrentLimit(MotorControllers.SMART_CURRENT_LIMIT);
    rightRear.setSmartCurrentLimit(MotorControllers.SMART_CURRENT_LIMIT);
    
    //creates a new diffdrive
    diffDrive = new DifferentialDrive(leftFront, rightFront); 
    diffDrive.setSafetyEnabled(false);
    diffDrive.setDeadband(DriveConstants.DEADBAND);

    //external encoders
    leftEncoder = new Encoder(DriveConstants.DIO_LDRIVE_ENC_A, DriveConstants.DIO_LDRIVE_ENC_B); 
    rightEncoder = new Encoder(DriveConstants.DIO_RDRIVE_ENC_A, DriveConstants.DIO_RDRIVE_ENC_B); 

    rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE_K);
    leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE_K);

    //pneumatic double solenoid
    transmission = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.SOL_LOW_GEAR, DriveConstants.SOL_HIGH_GEAR);
}


//methods start here
	public void setGearHigh(){
	transmission.set(Value.kReverse);
	}
	
	public void setGearLow(){
	transmission.set(Value.kForward);
	}
	
	public boolean isInLowGear(){
	return transmission.get() == Value.kForward;
	}
	
	public void closedRampRate() {	  //time in seconds to go from 0 to full throttle
	leftFront.setClosedLoopRampRate(MotorControllers.CLOSED_RAMP_RATE); 
	rightFront.setClosedLoopRampRate(MotorControllers.CLOSED_RAMP_RATE);
	}
	public void openRampRate() {
	leftFront.setClosedLoopRampRate(MotorControllers.OPEN_RAMP_RATE);
	rightFront.setClosedLoopRampRate(MotorControllers.OPEN_RAMP_RATE);
	}
	
	public void setLeftSpeed(double speed) {
	leftFront.set(speed);
	}
	
	public void setRightSpeed(double speed) {
	rightFront.set(speed);
	}
	
	public void setBothSpeeds(double speed) {
	leftFront.set(speed);
	rightFront.set(speed);
	}
	
	public void setTurnCWSpeeds(double speed) {
	leftFront.set(speed);
	rightFront.set(-speed);
	}
	
	public void setTurnCCWSpeeds(double speed) {
	leftFront.set(-speed);
	rightFront.set(speed);
	}
	
	public double getLeftSpeed(){
	//return leftEncoder.getVelocity(); //use for internal SparkMax encoder?
	
	//getRate units are distance per second, as scaled by the value of DistancePerPulse
		return leftEncoder.getRate(); //use for external drive encoders
	}
	
	public double getRightSpeed(){
		return rightEncoder.getRate(); //use for external drive encoders
	}
	
	public double getLeftEncoder() {
		return leftEncoder.getRaw();
	}
	
	public double getRightEncoder() {
		return rightEncoder.getRaw();
	}
	
	public double getLeftDistance() {
		return getLeftEncoder() * DriveConstants.DISTANCE_PER_PULSE_K;
		// distance per pulse * encoder reading = inches
	}
	
	public double getRightDistance() {
		//return rightEncoder.getDistance();
		return getRightEncoder() * DriveConstants.DISTANCE_PER_PULSE_K;
	}

	public double getAvgDistance() {
		return (getLeftDistance() + getRightDistance())/2 ;
	}
  
	public void resetLeftEncoder() {
    leftEncoder.reset();
	}
  
  
	public void resetRightEncoder() {
		rightEncoder.reset();
    }

	public void stop() {
		leftFront.set(0);
		rightFront.set(0);
	}
	
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

	SmartDashboard.getBoolean("In low gear?", isInLowGear());														 
  }
}
