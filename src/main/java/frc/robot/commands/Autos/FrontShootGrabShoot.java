// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Cartridge.PIDCartridgeShot;
import frc.robot.commands.Cartridge.PIDCartridgeTilt;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;


public class FrontShootGrabShoot extends SequentialCommandGroup {
  /** Creates a new FrontShootGrabShoot. */
  public FrontShootGrabShoot(Intake intake, Cartridge cartridge, Drive drive, double intSpeed, double cartSpeed, double drvDistance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //This command starts in front of the woofer, sets cartridge to woofer, shoots, (drives, runs intake, sets cartridge to podium at the same time), then shoots again
    addCommands(
      //new ToWooferPosition(cartridge), //TODO determine time for all below
      new PIDCartridgeTilt(cartridge, Constants.CartridgeShooter.TILT_ENC_REVS_WOOFER, Constants.CartridgeShooter.KP_TILT,  Constants.CartridgeShooter.KI_TILT,  Constants.CartridgeShooter.KD_TILT),
      new WaitCommand(2),
      new PIDCartridgeShot(intake, cartridge, intSpeed, cartSpeed, true).withTimeout(2), 
      new WaitCommand(1),
    Commands.parallel(
      new AutoPIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2), 
      new SetIntakeSpeed(intake, intSpeed),
      new PIDCartridgeTilt(cartridge, Constants.CartridgeShooter.TILT_ENC_REVS_STOW, Constants.CartridgeShooter.KP_TILT,  Constants.CartridgeShooter.KI_TILT,  Constants.CartridgeShooter.KD_TILT)
      //new ToPodiumPosition(cartridge)
      ),
      new WaitCommand(2),
      new PIDCartridgeShot(intake, cartridge, intSpeed, cartSpeed, false).withTimeout(5)
    );

  }
}
