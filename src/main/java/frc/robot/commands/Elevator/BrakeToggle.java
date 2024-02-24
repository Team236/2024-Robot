// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

public class BrakeToggle extends Command {
  private Elevator elevator;
  private boolean toggle;
  private boolean isBrakeEngaged;
  /** Creates a new ToggleGear. */
  public BrakeToggle(Elevator elevator) {
  this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //if in low gear change to high gear and set toggle true
  //otherwise change to low gear and set toggle true
  //comand is finised when toggle is true 
  toggle = false;

  if (elevator.isBrake()) {
    elevator.removeBrake();
    isBrakeEngaged = false;
  } else {
    elevator.engageBrake();
    isBrakeEngaged = true;
  }
  
  toggle = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Elev. Brake Engaged?:  ", isBrakeEngaged);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toggle;
  }
}

