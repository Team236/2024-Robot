// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  public CANSparkMax leftFront, leftRear, rightFront, rightRear;
  public DifferentialDrive diffDrive;
  //these are external encoders not SparkMAX
  private Encoder leftEncoder, rightEncoder;
//  private XboxController xboxController;

  /** Creates a new Drive. */
  public Drive() {
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

    leftFront.setSmartCurrentLimit(40);
    rightFront.setSmartCurrentLimit(40);
    leftRear.setSmartCurrentLimit(40);
    rightRear.setSmartCurrentLimit(40);


    DifferentialDrive diffDrive = new DifferentialDrive(leftFront, rightFront);
    

    //leftEncoder = new Encoder(Constants.DriveConstants.DIO_LDRIVE_ENC_A, Constants.DriveConstants.DIO_LDRIVE_ENC_B); 
    //rightEncoder = new Encoder(Constants.DriveConstants.DIO_RDRIVE_ENC_A, Constants.DriveConstants.DIO_RDRIVE_ENC_B); 

    //rightEncoder.setDistancePerPulse(Constants.DriveConstants.DISTANCE_PER_PULSE_K);
    //leftEncoder.setDistancePerPulse(Constants.DriveConstants.DISTANCE_PER_PULSE_K);
  }

  //methods start here
public void closedRampRate() {
  leftFront.setClosedLoopRampRate(0.08);
  rightFront.setClosedLoopRampRate(0.08);
}
public void openRampRate() {
  leftFront.setClosedLoopRampRate(0.08);
  rightFront.setClosedLoopRampRate(0.08);
}

public void stop() {
  leftFront.set(0);
  rightFront.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
