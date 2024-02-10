// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeShot;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WooferLeft extends SequentialCommandGroup {
  /** Creates a new RedLeft. */
  public WooferLeft(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, double intSpeed, double cartSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new AutoPIDTurn(drive, 180).withTimeout(2),
      //new AutoPIDTurn(drive, -180).withTimeout(2),
      new PIDCartridgeShot(intake, cartridge, tilt, intSpeed, cartSpeed, true).withTimeout(Constants.DriveConstants.CARTRIDGE_SHOOT_TIMEOUT), //fix
      new WaitCommand(1), //just for testing
      new AutoPIDDrive(drive, Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(2),
      new AutoPIDTurn(drive, -Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(2),
    Commands.parallel(
      new AutoPIDDrive(drive, Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
      new SetIntakeSpeed(intake, intSpeed).withTimeout(2)
      ),
      new AutoPIDDrive(drive, -Constants.DriveConstants.PULL_AWAY_TO_NOTE).withTimeout(2),
      new AutoPIDTurn(drive, Constants.DriveConstants.TURN_SIDE_OF_WOOFER).withTimeout(2),
      new AutoPIDDrive(drive, -Constants.DriveConstants.WOOFER_PULL_AWAY).withTimeout(2),
      new WaitCommand(1), //just for testing
      new PIDCartridgeShot(intake, cartridge, tilt, intSpeed, cartSpeed, true).withTimeout(Constants.DriveConstants.CARTRIDGE_SHOOT_TIMEOUT) //fix
    );
  }
}
