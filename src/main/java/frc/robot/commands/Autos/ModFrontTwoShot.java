// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.concurrent.ForkJoinWorkerThread;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartMotorsWooferOnly;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;


public class ModFrontTwoShot extends ParallelCommandGroup {

  public ModFrontTwoShot(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, Elevator elevator) {
    addCommands(
    new FrontTwoShots(intake, tilt, drive, elevator).withTimeout(16),
    new PIDCartMotorsWooferOnly(cartridge, Constants.CartridgeShooter.WOOFER_PID_LEFT_RPM, Constants.CartridgeShooter.WOOFER_PID_RIGHT_RPM).withTimeout(16)  //run cart motors in parallel with every command in Auto
    );
  }
}
 