// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import javax.sound.midi.Sequence;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartMotorsWooferOnly;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeShortWait;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PIDCartShotShtWaitWoofOnly extends ParallelCommandGroup {
  //Moves cartridge to Woofer or Podium position, then runs Cartridge at PID controlled velocity, then adds intake motors after a delay
  //intake speed between -1 and 1, cartridge speed in RPM
  public PIDCartShotShtWaitWoofOnly(Intake intake, Cartridge cartridge, Tilt tilt, double intSpeed, double cartSpeedLeft, double cartSpeedRight, double desiredRevs) {
      addCommands(
        new ManualIntakeShortWait(intake, intSpeed).withTimeout(2.3), //use manualIntake since counter =1 here
        new PIDCartMotorsWooferOnly(cartridge, cartSpeedLeft, cartSpeedRight).withTimeout(2.3),
        new PIDCartridgeTilt(tilt, desiredRevs).withTimeout(2.3)
        );
    Intake.resetCounter();  //reset counter after shooting a Note
  }

}