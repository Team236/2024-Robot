// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCCW;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class C_BlueToMidfield_3 extends ParallelCommandGroup {
  /** Creates a new C_BlueToMidfield_3. */
  public C_BlueToMidfield_3(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {

    addCommands(
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2),//bring elevator to match height (Start elev at bot limit at match start)
      Commands.sequence(
        new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER), //times out in 2 sec. Enough for tilt/then intake? Can reduce?
        Commands.parallel(
        new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2),
        new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2),
        new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(2)     
        ),
        //new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE), //times out 2 secs
        new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1), //shoots the Note
        new PIDTurnCCW(drive, 90).withTimeout(1.5),
        Commands.parallel(
        new PIDDrive(drive,  Constants.DriveConstants.NOTE_TO_NOTE).withTimeout(2),
        new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2),
        new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(2)         
        ),
        new PIDTurnCW(drive, 90-Constants.DriveConstants.TURN_ANGLE_BLUE_POD_TO_SPKR).withTimeout(1.5), 
        //new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_PODIUM),  //2 sec timeout
        new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1), //shoots the Note
        new PIDTurnCCW(drive,  90-Constants.DriveConstants.TURN_ANGLE_BLUE_POD_TO_SPKR).withTimeout(1.5), 
        new PIDDrive(drive, Constants.DriveConstants.NOTE_TO_MIDFLD).withTimeout(3)
       )
    );
    drive.setGearHigh();
    Intake.resetCounter();
  }
}