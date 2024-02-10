// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class OdometryDrive extends SubsystemBase {
  /** Creates a new OdometryDrive. */
  Encoder encoderLeft, encoderRight;
  CANSparkMax leftFront, rightFront, leftFollower, rightFollower;

  public OdometryDrive() {
    leftFront = new CANSparkMax(MotorControllers.ID_LEFT_FRONT,MotorType.kBrushless);
    rightFront = new CANSparkMax(MotorControllers.ID_RIGHT_FRONT,MotorType.kBrushless);
    leftFollower = new CANSparkMax(MotorControllers.ID_LEFT_REAR,MotorType.kBrushless);
    rightFollower = new CANSparkMax(MotorControllers.ID_RIGHT_REAR,MotorType.kBrushless);

    encoderLeft = new Encoder(DriveConstants.DIO_LDRIVE_ENC_A, DriveConstants.DIO_LDRIVE_ENC_B);
    encoderRight = new Encoder(DriveConstants.DIO_RDRIVE_ENC_A, DriveConstants.DIO_RDRIVE_ENC_B);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
