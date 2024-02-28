// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shots.AmpShot;
import frc.robot.commands.Shots.ClimbTrapShot;
import frc.robot.commands.Shots.WaitThenAmpShot;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbPID extends SequentialCommandGroup {
  /** Creates a new PIDElevClim. */

  public ClimbPID(Elevator elevator, AmpTrap ampTrap, Intake intake, Tilt tilt, Cartridge cartridge) {
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
      //new PIDDownToHeight(elevator, Constants.Elevator.MIN_HEIGHT + 10),
      new WaitCommand(5),
    //new PIDUptoHeight(elevator, Constants.Elevator.MIN_HEIGHT + 12).withTimeout(1),
    new BrakeEngage(elevator)
    );
  }
}
