// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class WooferRight extends SequentialCommandGroup {
  /** Creates a new WooferRight. */
  public WooferRight(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator, double intSpeed, double cartSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    Commands.parallel(
       //TODO Determine if elevator stays at match height - or need to hold the PID  
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2), //bring elevator up to match height
      new PIDCartridgeShot(intake, cartridge, tilt, intSpeed, cartSpeed, true).withTimeout(Constants.DriveConstants.CARTRIDGE_SHOOT_TIMEOUT) //fix
      ), 
      new WaitCommand(1), //just for testing
      new AutoPIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY),
      new AutoPIDTurn(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER),
    Commands.parallel(
      new AutoPIDDrive(drive, Constants.DriveConstants.PULL_AWAY_TO_NOTE),
      new IntakeWithCounter(intake, intSpeed)
      ),
      new AutoPIDDrive(drive, -Constants.DriveConstants.PULL_AWAY_TO_NOTE),
      new AutoPIDTurn(drive, -Constants.DriveConstants.TURN_SIDE_OF_WOOFER),
      new AutoPIDDrive(drive, -Constants.DriveConstants.WOOFER_PULL_AWAY),
      new WaitCommand(1), //just for testing
      new PIDCartridgeShot(intake, cartridge, tilt, intSpeed, cartSpeed, true).withTimeout(Constants.DriveConstants.CARTRIDGE_SHOOT_TIMEOUT) //fix
      //TODO Determine if line below needed - to hold elev with PID during teleop
      //,new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT)
      );
  }
}
