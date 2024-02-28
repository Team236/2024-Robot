// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class PIDDownToHeight extends Command {
  private Elevator elevator;
  private double desiredHeight; //desired height in inches
  private final PIDController pidController;
  private double kP = Constants.Elevator.KP_ELEV_DOWN;
  private double kI = Constants.Elevator.KI_ELEV_DOWN;
  private double kD = Constants.Elevator.KD_ELEV_DOWN;
  
  /** Creates a new SetElevatorHeight. */
  public PIDDownToHeight(Elevator elevator, double desiredHeight) {
    pidController = new PIDController(kP, kI, kD);
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    pidController.setSetpoint(desiredHeight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.removeBrake();
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    elevator.setElevSpeed(pidController.calculate(elevator.getElevatorHeight()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.isEBotLimit());
  }
}