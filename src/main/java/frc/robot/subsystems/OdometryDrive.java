// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorControllers;
import frc.robot.LimelightHelpers;

public class OdometryDrive extends SubsystemBase {

	private CANSparkMax leftFront, leftRear, rightFront, rightRear;
	private DifferentialDrive diffDrive;
	private DifferentialDriveOdometry diffDriveOdometry; 

	private DoubleSolenoid transmission;

	//these are external encoders not SparkMAX
	private Encoder leftEncoder, rightEncoder;
	private AHRS gyro;
	
		/** Creates a new OdometryDrive. */				 
	public OdometryDrive() {
		leftFront = new CANSparkMax(Constants.MotorControllers.ID_LEFT_FRONT, MotorType.kBrushless);
    	leftRear = new CANSparkMax(Constants.MotorControllers.ID_LEFT_REAR, MotorType.kBrushless);
    	rightFront = new CANSparkMax(Constants.MotorControllers.ID_RIGHT_FRONT, MotorType.kBrushless);
    	rightRear = new CANSparkMax(Constants.MotorControllers.ID_RIGHT_REAR, MotorType.kBrushless);

    	leftFront.restoreFactoryDefaults();
    	rightFront.restoreFactoryDefaults();

    	leftFront.setInverted(false);
    	rightFront.setInverted(true); //determine via bench testing
		
    	leftRear.follow(leftFront);
    	rightRear.follow(rightFront);

    	leftFront.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    	rightFront.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    		leftRear.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    		rightRear.setSmartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
    
		gyro = new AHRS();

    //creates a new diffdrive
    diffDrive = new DifferentialDrive(leftFront, rightFront); 
    	diffDrive.setSafetyEnabled(false);
    	diffDrive.setDeadband(DriveConstants.DEADBAND);

	diffDriveOdometry = new DifferentialDriveOdometry(
		gyro.getRotation2d(), 			//  gyro angle as Rotation2D 
		leftEncoder.getDistance(), 		//  encoder left distance
		rightEncoder.getDistance());  	//  encoder right distance
    
	//external encoders
    leftEncoder = new Encoder(DriveConstants.DIO_LDRIVE_ENC_A, DriveConstants.DIO_LDRIVE_ENC_B); 
    rightEncoder = new Encoder(DriveConstants.DIO_RDRIVE_ENC_A, DriveConstants.DIO_RDRIVE_ENC_B); 

	// TODO check that the distance per pulse is valid for 4 inch wheels
    rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE_K);
    leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE_K);

    	//pneumatic double solenoid
    transmission = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.SOL_LOW_GEAR, DriveConstants.SOL_HIGH_GEAR);
}

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
	SmartDashboard.getBoolean("In low gear?", isInLowGear());

	// Update the odometry in the periodic block
	diffDriveOdometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());								 
  }


//other methods start here
public Pose2d getPose() {
	return diffDriveOdometry.getPoseMeters();
	}

	// TODO determine what method to use for sparkmax to controll max output?
	/**  
	* Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
	* @param maxOutput the maximum output to which the drive will be constrained
	*/


	/**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds.
   */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    	return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
	}


	public void setGearHigh(){ transmission.set(Value.kReverse);	}

	public void setGearLow(){ transmission.set(Value.kForward);	}

	public boolean isInLowGear(){ return transmission.get() == Value.kForward;	}

	public void closedRampRate() {	  //time in seconds to go from 0 to full throttle
	leftFront.setClosedLoopRampRate(MotorControllers.CLOSED_RAMP_RATE); 
	rightFront.setClosedLoopRampRate(MotorControllers.CLOSED_RAMP_RATE);
	}
	public void openRampRate() {
	leftFront.setClosedLoopRampRate(MotorControllers.OPEN_RAMP_RATE);
	rightFront.setClosedLoopRampRate(MotorControllers.OPEN_RAMP_RATE);
	}
	
	/**
	 * @return
	 */
	public void setLeftSpeed(double speed) { leftFront.set(speed); }
	
	 /**
	 * @return
	 */
	public void setRightSpeed(double speed) { rightFront.set(speed); }
	
	/**
	 * @return
	 */
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
	
/**
* @return double 
*/
//getRate units are distance per second, as scaled by the value of DistancePerPulse
	public double getLeftSpeed(){ return leftEncoder.getRate();	}

	/**
	 * @return double
	 */
	//use for external drive encoders
	public double getRightSpeed(){ return rightEncoder.getRate(); 
	public Encoder getLeftEncoder(){ return leftEncoder;}
	public Encoder getRightEncoder(){ return rightEncoder;}

	// distance per pulse * encoder reading = inches
	public double getLeftDistance() { return leftEncoder.getDistance(); }
	public double getRightDistance() { return rightEncoder.getDistance(); }

	public double getAvgDistance() { return (getLeftDistance() + getRightDistance())/2 ; }
	public void resetLeftEncoder() { leftEncoder.reset(); }
  	public void resetRightEncoder() { rightEncoder.reset(); }


	public void zeroHeading() {	gyro.reset();}

	public void stop() {
	leftFront.set(0);
	rightFront.set(0);
	}

	public void setMaxOutput(double maxOutput) {
	diffDrive.setMaxOutput(maxOutput);
	  }


  /**
   * Drives the robot using arcade controls.
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
 */
public void ArcadeDrive(double fwd, double rot) {
	diffDrive.arcadeDrive(fwd, rot);
}

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDiveVolts(double leftVolts, double rightVolts) {
    leftFront.setVoltage(leftVolts);
    rightFront.setVoltage(rightVolts);
    diffDrive.feed();
  }

/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() { 
		leftEncoder.reset(); 
		rightEncoder.reset();
	}

/**
 * @return the turn rate of the robot in drees per second 
 *  in Clockwise direction?
 */
public double getTurnRate() {
	return -gyro.getRate();
}

public void resetOdometry(Pose2d pose){
	 // this seems to be valid
	diffDriveOdometry.resetPosition(gyro.getRotation2d(),0,0,pose);
}

/**
   * Resets the odometry to limelight if tag is not zero
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    resetEncoders();
	if (LimelightHelpers.getFiducialID("limelight") != 0 ) {
    	diffDriveOdometry.resetPosition( 
			gyro.getRotation2d(),
			0,
			0,
			LimelightHelpers.getBotPose2d("limelight") );
		}
	}
}
