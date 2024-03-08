// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.commands.Shots.PIDSpkrShotNoCart;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneWooferShot extends ParallelCommandGroup {
  /** Creates a new OneWooferShot. */
  public OneWooferShot(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
      new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT).withTimeout(3),//bring elevator to match height (Start elev at bot limit at match start)
      new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(5)
      //deleted line below, since ModOneWooferShot runs cart motors in parallel for the entire Auto
     // new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(3)
    );
    //drive.setGearHigh();
    Intake.resetCounter();
  }
}

