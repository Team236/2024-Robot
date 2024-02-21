// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Elevator.PIDtoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WooferLeft extends SequentialCommandGroup {
  /** Creates a new RedLeft. */
  public WooferLeft(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    Commands.parallel( 
        new PIDtoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2), //bring elevator up to match height
        new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2)
        ),
    new WaitCommand(1), //just for testing
    new AutoPIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(2),
    new AutoPIDTurn(drive, -Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(2),
    Commands.parallel(
        new AutoPIDDrive(drive, Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(3),
        new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(3)     
        ),
  //TODO: CAN CHANGE TO SHOT FROM FAR (with one Turn first) WITH NO DRIVING REVERSE?
    new AutoPIDDrive(drive, -Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
    new AutoPIDTurn(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(2),
    new AutoPIDDrive(drive, -Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(2),
    new WaitCommand(1), //just for testing
    new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(3)
    //TODO Determine if line below needed - to hold elev with PID during teleop
    // ,new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT)
      );
  }
}
