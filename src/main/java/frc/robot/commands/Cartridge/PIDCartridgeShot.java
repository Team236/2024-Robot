// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PIDCartridgeShot extends SequentialCommandGroup {
  /** Creates a new PIDCartridgeShot. */
  //intake speed should be between -1 and 1, cartridge speed should be in RPM
  public PIDCartridgeShot(Intake intake, Cartridge cartridge, double intSpeed, double cartSpeed, boolean isWoofer) {

    if (isWoofer) {
      addCommands(
        new ToWooferPosition(cartridge),
        Commands.parallel(
          new ManualIntake(intake, intSpeed).withTimeout(2), //determine timeout
          new PIDCartridgeMotors(cartridge, cartSpeed).withTimeout(2)
        )
      );
    } 
      else{
        addCommands(
        new ToPodiumPosition(cartridge),
        Commands.parallel(
          new ManualIntake(intake, intSpeed).withTimeout(2), //determine timeout
          new PIDCartridgeMotors(cartridge, cartSpeed).withTimeout(2)
        ) 
      );
    }
  }
}
