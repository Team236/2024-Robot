// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;
import java.util.concurrent.ForkJoinWorkerThread;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartMotorsWooferOnly;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCCW;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;


public class L_BlueToMidfield_2 extends ParallelCommandGroup {
  /** Creates a new ModWooferLeft. */
  public L_BlueToMidfield_2(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
    new PIDCartMotorsWooferOnly(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(5),
    Commands.sequence(
      Commands.parallel( 
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(2), //bring elevator up to match height
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2)
      ),
     new PIDDrive(drive, 165).withTimeout(3), //replaced with 2 lines below, to avoid skidding
      //new PIDDrive(drive, 83).withTimeout(2),
     // new PIDDrive(drive, 82).withTimeout(2),
      new PIDTurnCCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER- 5.3).withTimeout(1.5),
      Commands.parallel(
       new PIDDrive(drive, 200).withTimeout(2.5),
       new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2.5),
       new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.5)  
      ),
       Commands.parallel(
       new PIDDrive(drive, -20).withTimeout(2),
       new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2)
       )
    )
    );
  }
}