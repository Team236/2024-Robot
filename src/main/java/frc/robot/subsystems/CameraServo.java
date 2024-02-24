// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraServo extends SubsystemBase {

  public Servo cameraServo;
  private boolean isFloor;
  /** Creates a new CameraServo. */
  public CameraServo() {
    cameraServo = new Servo(Constants.PWM_FRONT_CAM);
  }

  public boolean isFloor() {
    return isFloor;
  }

  public void setIsFloor(boolean val) {
    isFloor = val;
  }

  public void setAngle(double angle) {
    cameraServo.set(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
