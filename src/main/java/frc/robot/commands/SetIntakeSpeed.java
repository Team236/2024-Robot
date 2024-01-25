// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends Command {

  private Intake intake;
  private double speed;
  private Counter counter;

  /** Creates a new SetIntakeSpeed. */
  public SetIntakeSpeed(Intake intake, double speed) {

    this.speed = speed;
    this.intake = intake;
    counter = new Counter();

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
  //  SmartDashboard.put("Intake count is:", Robot.); TODO fix dashboard statement
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean haveNote;
    if (counter.get() == 0) {
      haveNote = false;
    } else {
      haveNote = true;
    }
    return haveNote;
  }
}
