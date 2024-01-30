// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Cartridge.PIDCartridgeShot;
import frc.robot.commands.Cartridge.ToPodiumPosition;
import frc.robot.commands.Cartridge.ToWooferPosition;
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
      new ToWooferPosition(cartridge).withTimeout(0.5), //TODO determine time for all below
      new WaitCommand(2),
      new PIDCartridgeShot(intake, cartridge, intSpeed, cartSpeed, true).withTimeout(2), 
    Commands.parallel(
      new AutoPIDDrive(drive, Constants.DriveConstants.WOOFERFRONT_TO_NOTE).withTimeout(2), 
      new SetIntakeSpeed(intake, intSpeed),
      new ToPodiumPosition(cartridge).withTimeout(1)
      ),
      new WaitCommand(2),
      new PIDCartridgeShot(intake, cartridge, intSpeed, cartSpeed, false).withTimeout(5)
    );

  }
}
