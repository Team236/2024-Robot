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
import frc.robot.commands.Shots.PIDCartShotShortWait;
import frc.robot.commands.Shots.PIDCartShotShtWaitWoofOnly;
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
     // new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2),//elevator to match height (Start elev at bot limit at match start)
      Commands.sequence(
       // new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.3),
      new PIDCartShotShtWaitWoofOnly(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(1.5),
        Commands.parallel(
          new PIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(1.2), 
          new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.2),
          new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.2)     
          ),
        //******IF CENTER NOTE SHOT IS WIMPY, TRY LINE BELOW IN PLACE OF PIDCartridgeTilt/ManualIntake IN PARLLEL BELOW, TO INSERT DELAY BEFORE SHOT*******
        // new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(2.3),//2.3
       // Commands.parallel(
         // new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.7),
       //   new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.7) //shoots the Note
        //  ),
         new PIDCartShotShortWait (intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.5),
     
        new PIDTurnCCW(drive, 90).withTimeout(1),
        Commands.parallel(
          new PIDDrive(drive,  Constants.DriveConstants.NOTE_TO_NOTE).withTimeout(1.5), 
          new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(1.5),
          new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_CTR_NOTE).withTimeout(1.5)         
          ),
       // Commands.parallel(
        new PIDTurnCW(drive, 56).withTimeout(1), //*56
          //These commands are in parallel, so keep PIDSpkrShotNoCart because it has a 1 sec delay before shot - time enough to turn first
          ////new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_PODIUM+2).withTimeout(2.3)
          //),
        new PIDCartShotShortWait (intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_SIDE_NOTE_2).withTimeout(1.5),
        new PIDTurnCW(drive,  30).withTimeout(1.3), //25
        Commands.parallel(
         new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(3), 
         new PIDDrive(drive, Constants.DriveConstants.NOTE_TO_MIDFLD).withTimeout(3)
        ),
       new PIDDrive(drive, -150)
      )
    );
    Intake.resetCounter();
  }
}