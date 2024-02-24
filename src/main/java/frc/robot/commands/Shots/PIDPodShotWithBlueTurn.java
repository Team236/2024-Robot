// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Autos.AutoPIDTurn;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDPodShotWithBlueTurn extends SequentialCommandGroup {
//Moves cartridge to Woofer or Podium position, then runs Cartridge at PID controlled velocity, then adds intake motors after a delay
  //intake speed between -1 and 1, cartridge speed in RPM
  public PIDPodShotWithBlueTurn(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive) {
      addCommands(
          new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_BLUE_POD_TO_SPKR).withTimeout(1.5),
          new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_PODIUM)
      );
    Intake.resetCounter();  //reset counter after shooting a Note
  }

}