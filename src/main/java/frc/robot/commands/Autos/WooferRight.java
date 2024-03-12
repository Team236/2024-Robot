// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCCW;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;


public class WooferRight extends SequentialCommandGroup {

  public WooferRight(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
//****** ModWooferRight CALLS CARTRIDGE MOTORS IN PARALLEL - SO NO CART MOTORS USED HERE *********/
    addCommands(
    Commands.parallel( 
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2), //bring elevator up to match height
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3)
      ), 
    new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(1),
    new PIDTurnCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5),
    Commands.parallel(
      new PIDDrive(drive, Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
      new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2),
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(2)  
      ),
    new PIDDrive(drive, -Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(1.5),
    Commands.parallel(
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(1.5),
      new PIDTurnCCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5)
      ),
    Commands.parallel( //parallel since PIDSpkrshotNoCart has 1 sec delay before shot
      new PIDDrive(drive, -Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(2.3),
      //These commands are in parallel, so keep PIDSpkrShotNoCart because it has a 1 sec delay before shot - time enough to turn first
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3)
      )
    );
   drive.setGearHigh();
   Intake.resetCounter();
  }
}

