// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Shots.AmpShotNoCartMotors;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class R_Blue_2Speaker_1Amp extends ParallelCommandGroup {
  /** Creates a new R_Blue_2Speaker_1Amp. */
  public R_Blue_2Speaker_1Amp(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator, AmpTrap ampTrap) {

    addCommands(
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
     new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2),//bring elevator to match height (Start elev at bot limit at match start)
      Commands.sequence(
        new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER), //times out in 2 sec. Enough for tilt/then intake? Can reduce?
        new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(1),
        new PIDTurnCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5),
        new PIDDrive(drive,  Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
        new PIDTurnCCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5), //OR SLIGHTLY LESS ANGLE?
        new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER),  //2 sec timeout
        new PIDTurnCW(drive,  Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5), //OR SLIGHTLY LESS ANGLE?)
        new PIDDrive(drive, -37.9).withTimeout(3.5),
        new PIDTurnCW (drive, 90),
        new PIDDrive(drive, -28.6),
        new AmpShotNoCartMotors(intake, ampTrap, tilt)
       )
    );
    drive.setGearHigh();
  }
}
  
