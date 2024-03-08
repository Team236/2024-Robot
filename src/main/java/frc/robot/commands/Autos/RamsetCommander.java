// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.PATH;
import frc.robot.subsystems.Drive;

public class RamsetCommander extends Command {
  private Drive drive;
  private Trajectory currentTrajectory;
  RamseteCommand ramseteCommand;
  //public static final DifferentialDriveKinematics diffDriveKinematics;

  /** Creates a new RamsertCommander. */
  public RamsetCommander(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);

    // THIS IS STILL IN CONSTANTS , might move here later
    //diffDriveKinematics = new DifferentialDriveKinematics(Constants.PATH.trackWidthMeters);

var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
            PATH.ksVolts
          , PATH.kvVoltSecondsPerMeter
          , PATH.kaVoltSecondsSquaredPerMeter)
      , Constants.PATH.diffDriveKinematics      // might move this later to RamsetCommander
      ,10);

 // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
                PATH.kMaxSpeedMetersPerSecond,
                PATH.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(PATH.diffDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    currentTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass config
            config);

    ramseteCommand = new RamseteCommand(
            currentTrajectory,
            drive::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
                PATH.ksVolts,
                PATH.kvVoltSecondsPerMeter,
                PATH.kaVoltSecondsSquaredPerMeter),
            PATH.diffDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(PATH.kPDriveVel, 0, 0),
            new PIDController(PATH.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive
            );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
