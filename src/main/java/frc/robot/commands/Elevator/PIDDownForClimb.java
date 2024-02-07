// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class PIDDownForClimb extends Command { 
  private Elevator elevator;
  private double desiredHeight; //desired height in inches

public PIDDownForClimb(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight*Constants.Elevator.ELEV_IN_TO_REV;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //!!!! pidController.reset();
    elevator.setP(Constants.Elevator.KP_ELEV_CLIMB);
    elevator.setI(Constants.Elevator.KI_ELEV_CLIMB);
    elevator.setD(Constants.Elevator.KD_ELEV_CLIMB);
    elevator.setFF(Constants.Elevator.KFF_ELEV_CLIMB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    elevator.setSetpoint(desiredHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }
}