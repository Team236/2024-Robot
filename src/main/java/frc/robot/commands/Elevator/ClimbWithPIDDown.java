// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shots.ClimbTrapShot;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbWithPIDDown extends SequentialCommandGroup {
  /** Creates a new ClimbWithPIDDown. */
  public ClimbWithPIDDown(Elevator elevator, AmpTrap ampTrap, Intake intake, Tilt tilt, Cartridge cartridge) {
    addCommands( //assumes elevator starts at top
      Commands.parallel(
        Commands.sequence(
          new WaitCommand(0.5),
          new PIDDownToHeight(elevator, Constants.Elevator.JUST_ABOVE_CHAIN_HEIGHT).withTimeout(0.25)
        ),
        new ClimbTrapShot(intake, cartridge, ampTrap, tilt).withTimeout(0.01)
      ),
   Commands.parallel(
      new PIDDownToHeight(elevator, Constants.Elevator.MIN_HEIGHT).withTimeout(3),
      new ClimbTrapShot(intake, cartridge, ampTrap, tilt).withTimeout(3)
      ),
      new PIDUptoHeight(elevator, 3.5),//min height is 0.5"
      new WaitCommand(0.7),
      new PIDUptoHeight(elevator, 8),
      new WaitCommand(0.5),
      new PIDUptoHeight(elevator, 15), 
      new WaitCommand(0.5),
      new PIDUptoHeight(elevator, 21),
      new WaitCommand(0.4),
      new PIDUptoHeight(elevator, 22.5),//just above chain is 22.8" 
      new WaitCommand(0.2),
      new PIDUptoHeight(elevator, Constants.Elevator.MAX_HEIGHT) //max height is 27"
    );
  }
}

