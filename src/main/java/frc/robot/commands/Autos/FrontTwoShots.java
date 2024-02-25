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
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class FrontTwoShots extends SequentialCommandGroup {
  /** Creates a new FrontTwoShots. */
  public FrontTwoShots(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
    Commands.parallel(
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2),//bring elevator to match height (Start elev at bot limit at match start)
      new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(3)
      ),
    Commands.parallel(
      new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2),
      new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2)
      ),
    new PIDDrive(drive, -Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2),
    //new WaitCommand(1),
    //TODO: CAN CHANGE TO FAR SHOT WITH NO DRIVING REVERSE? ALWAYS MAKE FAR SHOT OR NOT?
    new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(3),
    new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(3)
    );
  }
}

