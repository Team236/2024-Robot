// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class PIDDownToHeight extends Command {
  private Elevator elevator;
  private double desiredHeight; //desired height in inches

public PIDDownToHeight(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //!!!! pidController.reset();
    elevator.setP(Constants.Elevator.KP_ELEV_DOWN);
    elevator.setI(Constants.Elevator.KI_ELEV_DOWN);
    elevator.setD(Constants.Elevator.KD_ELEV_DOWN);
    elevator.setFF(Constants.Elevator.KFF_ELEV_DOWN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    elevator.setSetpoint(desiredHeight*Constants.Elevator.ELEV_IN_TO_REV);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //added last contiditon to test if elevator holds using just the brake (PID ends 2% early)
    return (elevator.isEBotLimit() || elevator.getElevatorHeight() <= 1.02*desiredHeight);
  }
}
