// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FrontCamera extends SubsystemBase {
  private UsbCamera frontCamera;
  private Servo servo;
  private double angle;
  /** Creates a new FrontCamera. */
  public FrontCamera() {
    servo = new Servo(Constants.Camera.PWM_CAMERA);
  }
  // methods start here 
  //public void setAngle(angle);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
