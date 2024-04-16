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
import frc.robot.commands.Shots.AmpShot;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

//
public class AmpRedLeft extends SequentialCommandGroup {

//****** ModWooferLeft CALLS CARTRIDGE MOTORS IN PARALLEL - SO NO CART MOTORS USED HERE *********/
  public AmpRedLeft(Intake intake, Cartridge cartridge, AmpTrap ampTrap, Tilt tilt, Drive drive, Elevator elevator) {
    
    addCommands(
    new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(1), //bring elevator up to match height
      //new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3)
    new PIDDrive(drive, 19).withTimeout(1.25),
    new PIDTurnCW(drive, 90).withTimeout(1.5), //+3
    new PIDDrive(drive, 17).withTimeout(1.5),//7
    new AmpShot(intake, cartridge, ampTrap, tilt).withTimeout(5)
      );
   Intake.resetCounter();
  }
}

