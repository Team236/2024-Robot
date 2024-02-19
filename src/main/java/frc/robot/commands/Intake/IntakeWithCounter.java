// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeWithCounter extends Command {
  //runs the intake but stops when count is greater than zero

  private Intake intake;
  private double speed;

  /** Creates a new SetIntakeSpeed. */
  public IntakeWithCounter(Intake intake, double speed) {
    this.speed = speed;
    this.intake = intake;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // intake.resetCount(); //TODO NEED TO REMOVE THIS FOR TEST?
  }

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
    //stop intake motor when Note in robot (count > 0)
    //changed from intake.  to Intake.
    return Intake.getIntakeCount() > 0;

  }
  
}
