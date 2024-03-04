// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;


public class L_RedToMidfield_2 extends ParallelCommandGroup {

  public L_RedToMidfield_2(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {

    addCommands(
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
      Commands.sequence(
        new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER), //times out in 3 sec. Enough for tilt/then intake? Can reduce?
        new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(1),
        new PIDTurnCCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER + 2).withTimeout(1.5),
        new PIDDrive(drive,  Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
        new PIDTurnCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER + 2).withTimeout(1.5), //OR SLIGHTLY LESS ANGLE?
        new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER),  //3 sec timeout
        new PIDTurnCCW(drive,  Constants.DriveConstants.TURN_SIDE_OF_WOOFER + 2).withTimeout(1.5), //OR SLIGHTLY LESS ANGLE?)
        new PIDDrive(drive, Constants.DriveConstants.NOTE_TO_MIDFLD).withTimeout(3)
       )
    );
  }
}
  
 

