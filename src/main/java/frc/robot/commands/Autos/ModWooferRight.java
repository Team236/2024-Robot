// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

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
import frc.robot.commands.Shots.PIDCartShotShortWait;
import frc.robot.commands.Shots.PIDCartShotShtWaitWoofOnly;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ModWooferRight extends ParallelCommandGroup {
  /** Creates a new ModWooferRight. */
  public ModWooferRight(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
     // new PIDCartMotorsWooferOnly(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(16),  //run cart motors in parallel with every command in Auto
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(16),//bring elevator to match height (Start elev at bot limit at match start)
      Commands.sequence(
        new PIDCartShotShtWaitWoofOnly (intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(1.5),
       // new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(2.5),
        new PIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY + 1).withTimeout(1),
        new PIDTurnCW(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(1.5),
        Commands.parallel(
         new PIDDrive(drive,  Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
         new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2),
         new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(2)    
         ),    
        new PIDDrive(drive,  -6).withTimeout(1),
        new PIDTurnCCW(drive, 31).withTimeout(1),//next command has 1 sec wait, so there is time to turn //35 for cw and ccw
        new PIDCartShotShortWait (intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED,  Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM, Constants.Tilt.TILT_ENC_REVS_SIDE_NOTE).withTimeout(2),
        new PIDTurnCW(drive, 31).withTimeout(1.5),      
        new PIDDrive(drive,  -60).withTimeout(1) 
      )
        );
    Intake.resetCounter();
  }
}
   
   
   /* 
    addCommands(
    new WooferRight(intake, cartridge, tilt, drive, elevator).withTimeout(16),
    new PIDCartMotorsWooferOnly(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(16)  //run cart motors in parallel with every command in Auto
    );
  }
}
*/
