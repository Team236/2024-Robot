// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class PIDGyroTurnCW extends Command {
  private final Drive drive;
  private final PIDController gyroPidController;
  private AHRS navx;

  /** Creates a new PIDGyroTurnCW. */
  public PIDGyroTurnCW(Drive drive, AHRS navx, double setpointDegrees) {
      this.navx = navx;
      this.drive = drive;
      this.gyroPidController = new PIDController(Constants.DriveConstants.KP_GYRO_TURN_CW, 0, 0);
      gyroPidController.setSetpoint(setpointDegrees); 
      addRequirements(drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //gyro is mounted perp to ground - the "X" axis on the gyro is the Z (after calibration)
    navx.reset();//zeroes the Yaw (twist about the X-axis)
    gyroPidController.reset();
    drive.resetLeftEncoder();
    drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double speed = gyroPidController.calculate(navx.getYaw());
        drive.setLeftSpeed(speed);
        drive.setRightSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



