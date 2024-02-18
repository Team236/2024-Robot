// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

//!!!! import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class PIDUptoHeight extends Command {
  private Elevator elevator;
  private double desiredHeight; //desired height in inches
  //!!!!private final PIDController pidController;
  //!!!! private double eKP, eKI, eKD;

  /** Creates a new SetElevatorHeight. */
  //!!!! removed since using SparkMax PID:  
  //!!!!public SetElevatorHeight(Elevator elevator, double desiredHeight, double eKP, double eKI, double eKD) {
  //!!!!  this.eKP = eKP;
  //!!!! this.eKI = eKI;
  //!!!! this.eKD = eKD;
  //!!!!  pidController = new PIDController(eKP, eKI, eKD)
  public PIDUptoHeight(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    //!!!!pidController.setSetpoint(desiredHeight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //!!!! pidController.reset();
    elevator.setP(Constants.Elevator.KP_ELEV_UP);
    elevator.setI(Constants.Elevator.KI_ELEV_UP);
    elevator.setD(Constants.Elevator.KD_ELEV_UP);
    elevator.setFF(Constants.Elevator.KFF_ELEV_UP);
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    elevator.setSetpoint(desiredHeight*Constants.Elevator.ELEV_IN_TO_REV);
    //****elevator.setElevSpeed(pidController.calculate(elevator.getElevatorHeight()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    //added last contiditon to test if elevator holds using just the brake (PID ends 2% early)
    return (elevator.isTop() || elevator.isETopLimit());
  }
}
