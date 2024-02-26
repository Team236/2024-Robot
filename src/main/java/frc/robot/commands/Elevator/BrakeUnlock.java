
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
public class BrakeUnlock extends Command {
  
  //use "driveLG" not just "drive", so you can drive while changing gears
private Elevator elevator;
  /** Creates a new LowGear. */
  public BrakeUnlock(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
 addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.removeBrake();
   // new WaitCommand(1);  //To ensure no elevator command starts before brake disengaged
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //brake is in intializa, so returning true will release elevator subsystem 
  }
}