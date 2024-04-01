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

//
public class WooferLeft extends SequentialCommandGroup {

//****** ModWooferLeft CALLS CARTRIDGE MOTORS IN PARALLEL - SO NO CART MOTORS USED HERE *********/
  public WooferLeft(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
    Commands.parallel( 
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2), //bring elevator up to match height
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3)
      ),
    new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY+4).withTimeout(1.25),
    new PIDTurnCCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER-3).withTimeout(1.5), //+3
    Commands.parallel(
      new PIDDrive(drive, Constants.DriveConstants.PULL_AWAY_TO_NOTE-3).withTimeout(2.3),//-5
      new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2.3),
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3)  
      ),
    Commands.parallel(
       new PIDDrive(drive, -Constants.DriveConstants.PULL_AWAY_TO_NOTE+3).withTimeout(1.5),//7
       new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.5)
      ),
    Commands.parallel(
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(1.5),  
      new PIDTurnCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5)//+2
      ),
    Commands.parallel(  //parallel because Tilting in SpkrShot takes 1 second, before shooting)
      new PIDDrive(drive, -Constants.DriveConstants.WOOFER_PULL_AWAY-6).withTimeout(2.3),
      //These commands are in parallel, so keep PIDSpkrShotNoCart because it has a 1 sec delay before shot - time enough to turn first
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3)
      )
      );
   Intake.resetCounter();
  }
}

