// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends Command {

  private Intake intake;
  private double speed;
  private Robot robot;

  /** Creates a new SetIntakeSpeed. */
  public SetIntakeSpeed(Intake intake, Robot robot, double speed) {
    this.speed = speed;
    this.intake = intake;
    this.robot = robot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (robot.counter.get() > 0);
  }
  
}
