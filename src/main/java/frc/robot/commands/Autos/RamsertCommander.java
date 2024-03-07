// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class RamsertCommander extends Command {
  private Trajectory currentTrajectory;
  private DifferentialDriveKinematics kDriveKinematics;

  /** Creates a new RamsertCommander. */
  public RamsertCommander() {
    // Use addRequirements() here to declare subsystem dependencies.
    kDriveKinematics = new DifferentialDriveKinematics(Constants.PATH.kTrackwidthMeters);
  
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
