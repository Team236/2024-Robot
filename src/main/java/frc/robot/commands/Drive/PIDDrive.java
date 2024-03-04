// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class PIDDrive extends Command {
  private Drive drive;
  private double driveDistance;
  private final PIDController pidController;

  /** Creates a new AutoPIDDrive. */
  //this comand uses PID to drive a distance equal to drive distance in inches
  public PIDDrive(Drive drive, double driveDistance) {
    this.drive = drive;
    this.driveDistance = driveDistance;
    this.pidController = new PIDController(Constants.DriveConstants.KP_DRIVE, Constants.DriveConstants.KI_DRIVE, Constants.DriveConstants.KD_DRIVE);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    pidController.setSetpoint(driveDistance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    drive.resetLeftEncoder();
    drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //uses WPIPID and external encoder, not SPARKMAX encoder or SparkPID
    drive.setBothSpeeds(pidController.calculate(drive.getAvgDistance()));
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
   /* 
    boolean done;
    if (drive.getAvgDistance() > 0.97*driveDistance1) {
       done = true;
    }  
    else {done = false;
    }
    return done; 
    */
  }
}
