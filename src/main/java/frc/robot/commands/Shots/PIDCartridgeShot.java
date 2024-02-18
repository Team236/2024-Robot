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
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PIDCartridgeShot extends SequentialCommandGroup {
  //Moves cartridge to Woofer or Podium position, then runs intake at set speed and Cartridge at PID controlled velocity
  //intake speed should be between -1 and 1, cartridge speed should be in RPM
  public PIDCartridgeShot(Intake intake, Cartridge cartridge, Tilt tilt, double intSpeed, double cartSpeed, boolean isWoofer) {

    //runs wait and tilt in series, while running intake/cartridge in parallel
    //the wait makes the cartridge extend and hold extended, while the shot takes place after the wait
    if (isWoofer) {
      addCommands(
          new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER).withTimeout(3),
        Commands.parallel(
          new ManualIntake(intake, intSpeed).withTimeout(5)), //use manualIntake since counter =1 here
          new PIDCartridgeMotors(cartridge, cartSpeed).withTimeout(5));
    }
      else{
        addCommands(
          new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM).withTimeout(3),
        Commands.parallel(
          new ManualIntake(intake, intSpeed).withTimeout(5)), //use manualIntake since counter =1 here
          new PIDCartridgeMotors(cartridge, cartSpeed).withTimeout(5));
    }
    Intake.resetCounter();  //reset counter after shooting a Note
  }

}