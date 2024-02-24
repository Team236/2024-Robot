// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeShotLeftAngle extends SequentialCommandGroup {
  /** Creates a new ThreeShotLeftAngle. */
  public ThreeShotLeftAngle(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel( 
        new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2), //bring elevator up to match height
        new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(3)
        ),
      Commands.parallel(
        new PIDDrive(drive, Constants.DriveConstants.WOOFER_ANGLED_TO_NOTE).withTimeout(3),
        new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(3)     
        ),
      new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(3),
      new PIDTurn(drive, -Constants.DriveConstants.TURN_TO_THIRD_NOTE).withTimeout(2),
      Commands.parallel(      
          new PIDDrive(drive, -Constants.DriveConstants.NOTE_TO_NOTE).withTimeout(2),
          new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(3)     
      ),
        new PIDTurn(drive, 90).withTimeout(2),
        new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_AUTOSHOT2).withTimeout(3)
    );
  }
}
