// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.commands.Shots.PIDCartridgeShotWooferOnly;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class R_WooferShotPullOut extends ParallelCommandGroup {
  /** Creates a new OneWooferShot. */
  public R_WooferShotPullOut(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
    new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(3),//bring elevator to match height (Start elev at bot limit at match start) 
    //Runs Tilt and Cart motors, with intake motors starting after a 1 sec wait (so cart motors are at speed and cartridge is tilted before shot):
    new PIDCartridgeShotWooferOnly(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(5),
     
    new SequentialCommandGroup(
      new WaitCommand(11),
      new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY + 1).withTimeout(1),
      new PIDTurnCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5),
      new PIDDrive(drive,  Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2)
      )
    );
    Intake.resetCounter();
  }
}
 //new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(5),
 //new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(5)  /