// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CartridgeShooter;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;


public class FrontShootGrabShoot extends SequentialCommandGroup {
  /** Creates a new FrontShootGrabShoot. */
  public FrontShootGrabShoot(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator, double intSpeed, double cartSpeed, double drvDistance) {
    //This command starts in front of the woofer, sets cartridge to woofer, shoots, (drives, runs intake, sets cartridge to podium at the same time), then shoots again
    addCommands(
    Commands.parallel(
      new PIDCartridgeShot(intake, cartridge, tilt, intSpeed, cartSpeed, true).withTimeout(4), 
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2) //bring elevator up to match height
      ),
    new WaitCommand(1),
    Commands.parallel(
      new AutoPIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(4),
      new ManualIntake(intake, intSpeed).withTimeout(5),
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(5)
      ),
    new AutoPIDDrive(drive, -Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(4),
    new WaitCommand(2),
    new PIDCartridgeShot(intake, cartridge, tilt, intSpeed, cartSpeed, true).withTimeout(4)
    //TODO Determine if line below needed - to hold elev with PID during teleop
    // ,new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT)

    );

  }
}
