// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class PIDTurnCCW extends Command {
    private final Drive drive;
  private final PIDController leftPidController, rightPidController;

  /** Creates a new PIDTurnCCW. */
  public PIDTurnCCW(Drive drive, double setpointDegrees) {
    this.drive = drive;
    this.leftPidController = new PIDController(Constants.DriveConstants.KP_TURN_CCW, 0, 0);
    this.rightPidController = new PIDController(Constants.DriveConstants.KP_TURN_CCW, 0, 0);
    leftPidController.setSetpoint(-setpointDegrees * Constants.DriveConstants.TURNCCW_DEG_TO_INCHES); // this converts the setpoint in degrees to inches, need new conversion for this year
    rightPidController.setSetpoint(setpointDegrees * Constants.DriveConstants.TURNCCW_DEG_TO_INCHES);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPidController.reset();
    rightPidController.reset();
    drive.resetLeftEncoder();
    drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //uses WPIPID and external encoder, not SPARKMAX encoder or SparkPID
    double leftSpeed = leftPidController.calculate(drive.getLeftDistance());
    double rightSpeed = rightPidController.calculate(drive.getRightDistance());
    drive.setLeftSpeed(leftSpeed);
    drive.setRightSpeed(rightSpeed);
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
