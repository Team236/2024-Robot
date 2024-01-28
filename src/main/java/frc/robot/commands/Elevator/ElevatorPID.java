// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorPID extends Command {
  private Elevator elevator;
  private double setPoint; //travel distance in inches
  private final PIDController pidController;
  private double eKP, eKI, eKD;

  /** Creates a new UpDownPID. */
  public ElevatorPID(Elevator elevator, double setPoint, double eKP, double eKI, double eKD) {
    this.eKP = eKP;
    this.eKI = eKI;
    this.eKD = eKD;
    pidController = new PIDController(eKP, eKI, eKD);
    this.elevator = elevator;
    this.setPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    pidController.setSetpoint(setPoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();

    elevator.resetElevatorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevSpeed(pidController.calculate(elevator.getElevatorDistance()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
