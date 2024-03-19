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
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class FrontTwoShots extends SequentialCommandGroup {
  //****** ModFrontTwoShots CALLS CARTRIDGE MOTORS IN PARALLEL - SO NO CART MOTORS USED HERE *********/
  public FrontTwoShots(Intake intake, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(

    Commands.parallel(
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2),//bring elevator to match height (Start elev at bot limit at match start)
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3) 
      ),
    Commands.parallel(
      new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2),
      new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2),
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2)     
      ),
    Commands.parallel(
     new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(1.5), 
     new PIDDrive(drive, -Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(1.5),
     new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.5)
      ),
    //***IF WANT TO SPEED UP THIS SHOT, REPLACE PIDSpkrShotNoCart below with the four lines below
      // Commands.parallel(
      // new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER.withTimeout(1.7),
      // new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.7) //shoots the Note
      //),
    new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3),
    new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2)
    );
    Intake.resetCounter();
  }
}
