// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCCW;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;


public class PIDPodShotWithRedTurn extends ParallelCommandGroup {
//Moves cartridge to Woofer or Podium position, then runs Cartridge at PID controlled velocity, then adds intake motors after a delay
  //intake speed between -1 and 1, cartridge speed in RPM
  public PIDPodShotWithRedTurn(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive) {
    addCommands(
    new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM, Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM).withTimeout(4),
    new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(4),
    Commands.sequence(
          new PIDDrive(drive, -10).withTimeout(.7),
          new PIDTurnCCW(drive, Constants.DriveConstants.TURN_ANGLE_RED_POD_TO_SPKR).withTimeout(1),
          new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2)
         // new PIDSpkrShotNoCart(intake, tilt, Constants.Intake.INTAKE_SPEED, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(2)
    )
      );
    Intake.resetCounter();  //reset counter after shooting a Note
  }
}
