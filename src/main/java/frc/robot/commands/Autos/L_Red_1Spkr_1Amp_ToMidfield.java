// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCCW;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.AmpShot;
import frc.robot.commands.Shots.AmpShotNoCartMotors;
import frc.robot.commands.Shots.PIDCartShotShtWaitWoofOnly;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L_Red_1Spkr_1Amp_ToMidfield extends ParallelCommandGroup {
  /** Creates a new L_Red_1Spkr_1Amp_ToMidfield. */
  public L_Red_1Spkr_1Amp_ToMidfield(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator, AmpTrap ampTrap) {

    addCommands(
     // new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2),//bring elevator to match height (Start elev at bot limit at match start)
      Commands.sequence(
        new PIDCartShotShtWaitWoofOnly(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(1.5),
        //new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3), 
        new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY + 2).withTimeout(1),  //* +5
        new PIDTurnCCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5),
        Commands.parallel(
          new PIDDrive(drive,  Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2.5),
          new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2.5),
          new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(2.5)         
         ),
        new PIDDrive(drive, -33).withTimeout(1.5),
        new PIDTurnCW (drive, 95).withTimeout(1.5),
        Commands.parallel(
         new PIDDrive(drive, 32).withTimeout(1.75),
         new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(1.75)
         ),
        new AmpShot(intake, cartridge, ampTrap, tilt).withTimeout(2.5)  //lower this timeout?
        // ,new PIDTurnCW(drive, 90).withTimeout(1),
        // new PIDDrive(drive, 120)
       )
      );
    Intake.resetCounter();
  }
}
  
