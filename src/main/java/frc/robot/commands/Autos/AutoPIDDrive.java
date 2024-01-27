// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class AutoPIDDrive extends Command {
  private Drive drive;
  private double driveDistance1;
  private final PIDController pidController;

  /** Creates a new AutoPIDDrive. */
  public AutoPIDDrive(Drive drive, double driveDistance) {
    this.drive = drive;
    this.driveDistance1 = driveDistance;
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
   /* 
    boolean done;

    if (drive.getAvgDistance() > 0.97*driveDistance1) {
       done = true;
    }  
    else {done = false;
    }

    return done; 
    */
    return false;

  }
}
