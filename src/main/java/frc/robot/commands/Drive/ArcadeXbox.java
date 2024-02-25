// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ArcadeXbox extends Command {
  private Drive drive;
  private XboxController driverController;
  private boolean inLeftYDeadzone, inRightXDeadzone;
  private DifferentialDrive diffDrive;
   // public CANSparkMax leftFront, rightFront;
  private double leftSpeed, rightRotation;

  // Creates a new ArcadeXbox 
  //for driving Arcade style using 2 sticks (left/right) on a single XBox 
  public ArcadeXbox(DifferentialDrive diffDrive, XboxController driverController, Drive drive1)  {
  this.drive = drive1;
    this.driverController = driverController;
    this.diffDrive = diffDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   //TODO try without ramp rate, also try adding slew rate in driveArcade method
    drive.openRampRate();
    drive.resetLeftEncoder();
    drive.resetRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //speed is Y-axis of left stick, rotation is X-axis of right stick
    //speed is getLeftY, rotation is getRightX, for arcade drive with 2 sticks
    

  //TODO  - replace last line in execute with the 2 lines below and see effect
  //SlewRateLimiter filter = new SlewRateLimiter(0.5);
  // limits the rate of change of the signal to 0.5 units per second

  leftSpeed = -driverController.getLeftY();
  rightRotation = -driverController.getRightX();
  diffDrive.arcadeDrive(leftSpeed, rightRotation);
}

@Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
