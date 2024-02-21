// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shots.AmpShot;
import frc.robot.commands.Shots.WaitThenAmpShot;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDActualClimb extends SequentialCommandGroup {
  /** Creates a new PIDElevClim. */

  public PIDActualClimb(Elevator elevator, AmpTrap ampTrap, Intake intake, Tilt tilt, Cartridge cartridge) {
    addCommands( //assumes elevator starts at top
      new PIDDownToHeight(elevator, Constants.Elevator.JUST_ABOVE_CHAIN_HEIGHT).withTimeout(2),
      Commands.parallel(
         new PIDDownToHeight(elevator, Constants.Elevator.MIN_HEIGHT),
         new WaitThenAmpShot(intake, cartridge, ampTrap, tilt).withTimeout(5)
      )
    );

  
  }
}
