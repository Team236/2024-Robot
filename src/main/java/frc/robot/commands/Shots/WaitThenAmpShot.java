// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WaitThenAmpShot extends SequentialCommandGroup {
  /** Creates a new WaitThenAmpShot. */
  public WaitThenAmpShot(Intake intake, Cartridge cartridge, AmpTrap ampTrap, Tilt tilt) {
    addCommands(
      new WaitCommand(0.0),
      new AmpShot(intake, cartridge, ampTrap, tilt)
    );
  }
}
