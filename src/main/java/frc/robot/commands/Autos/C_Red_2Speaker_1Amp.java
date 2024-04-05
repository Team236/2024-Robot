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
import frc.robot.commands.Shots.PIDCartShotShortWait;
import frc.robot.commands.Shots.PIDCartShotShtWaitWoofOnly;
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
public class C_Red_2Speaker_1Amp extends ParallelCommandGroup {
  /** Creates a new L_Red_2Speaker_1Amp. */
  public C_Red_2Speaker_1Amp(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator, AmpTrap ampTrap) {

    addCommands(
     //new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
     new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(16),//bring elevator to match height (Start elev at bot limit at match start)
      Commands.sequence(
        //new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3),
        new PIDCartShotShtWaitWoofOnly(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(1.5),
        Commands.parallel(
        new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(1.75),
        new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.75),
        new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.75)     
        ),
        //******IF CENTER NOTE SHOT IS WIMPY, TRY LINE BELOW IN PLACE OF PIDCartridgeTilt/ManualIntake IN PARLLEL BELOW, TO INSERT DELAY BEFORE SHOT*******
        // new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.7),//2.3
        //Commands.parallel(
         // new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.7),
         // new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.7) //shoots the Note
        // ),
        new PIDCartShotShortWait (intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(2),
        new PIDTurnCW(drive, 90).withTimeout(1),
        Commands.parallel(
         new PIDDrive(drive, Constants.DriveConstants.NOTE_TO_NOTE + 3).withTimeout(1.75),
         new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.75),
         new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(1.75)          
        ),
        new PIDTurnCCW(drive, 85).withTimeout(1.2), 
        new PIDDrive(drive, -42).withTimeout(1.2), //*-45 */
        Commands.parallel(
        new PIDTurnCW (drive, 95).withTimeout(1),
        new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(1)
        ), 
        Commands.parallel(
        new PIDDrive(drive, 30).withTimeout(1),//*33 //in parallel shince there is a 1 sec delay before shot in next command
        new AmpShot(intake, cartridge, ampTrap, tilt).withTimeout(5)
        )
      )
    );
    Intake.resetCounter();
  }
}
      
 