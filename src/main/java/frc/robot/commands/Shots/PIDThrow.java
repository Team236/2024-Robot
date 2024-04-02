// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.PIDDrive;
import frc.robot.commands.Drive.PIDTurnCW;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

public class PIDThrow extends ParallelCommandGroup {
//This command throws the Note over the Stage (forward pass)
  public PIDThrow(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive) {
    addCommands(
    new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM).withTimeout(4),
    new PIDCartridgeTilt(tilt, -40).withTimeout(4)
   /*  Commands.sequence(
      new WaitCommand(0.25),
      new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2)
      )
      */
    ); 
    Intake.resetCounter();  //reset counter after shooting a Note
  }
}