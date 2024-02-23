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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class PIDPoseCommand extends Command {
  private Drive drive;
//   private double driveDistance1;
  private final PIDController pidController;

  //private final Odometry odometryDrive;
  private final RamseteCommand ramseteCommand;
  private final TrajectoryConfig config;
  private final edu.wpi.first.math.trajectory.Trajectory exampleTrajectory;

  /** Creates a new AutoPIDDrive. */
  //this comand uses PID to drive a distance equal to drive distance in inches
  public PIDPoseCommand(Drive drive) { 
    this.drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    // pidController.setSetpoint(?distance?);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    drive.resetEncoders();
    
       // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
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


   exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)) ,
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)) ,
            new Pose2d(3, 0, new Rotation2d(0)) ,
            config);

                RamseteCommand ramseteCommand = 
                new RamseteCommand( 
                    exampleTrajectory,
                    drive.getPose(),
                    new RamseteController(),
                    new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter ),
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                        drive::tankDiveVolts,
                    drive );

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.setBothSpeeds(pidController.calculate(drive.getAvgDistance()));


    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.

   return Commands.runOnce(() -> odometryDrive.resetOdometry(exampleTrajectory.getInitialPose()))
        .andThen(ramseteCommand)
        .andThen(Commands.runOnce(() -> odometryDrive.stop()));
    }


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
