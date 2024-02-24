// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class PIDPoseCommand extends Command {

  private Drive drive;
  private TrajectoryConfig config;
  
  public void PIDPoseCommand(Drive drive) { 
    this.drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),  
        DriveConstants.kDriveKinematics, 
        10);   // 10 volts declared as maximum 

      // Create config for trajectory
       config = new TrajectoryConfig(
                DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

      TrajectoryGenerator.generateTrajectory( 
          new Pose2d(0, 0, new Rotation2d(0)) 
        , List.of(new Translation2d(1, 1), new Translation2d(2, -1)) 
        ,  new Pose2d(3, 0, new Rotation2d(0)) 
        , config);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
       
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.setBothSpeeds(pidController.calculate(drive.getAvgDistance()));
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
