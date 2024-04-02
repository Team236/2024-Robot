// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shots.ClimbTrapShot;
import frc.robot.commands.Shots.ClimbTrapShotWithWait;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAtEnd extends SequentialCommandGroup {
  /** Creates a new ClimbWithManual. */
  public ClimbAtEnd(Elevator elevator, AmpTrap ampTrap, Intake intake, Tilt tilt, Cartridge cartridge) {
    addCommands( //assumes elevator starts at top
    //new WaitCommand(0.5),
    new PIDDownToHeight(elevator, Constants.Elevator.JUST_ABOVE_CHAIN_HEIGHT).withTimeout(0.25),
    Commands.parallel(
      new PIDDownToHeight(elevator, Constants.Elevator.CLIMB_HEIGHT).withTimeout(2), //TODO Change 3.75 to 2.75 or lower
      new ClimbTrapShotWithWait(intake, cartridge, ampTrap, tilt).withTimeout(2)
      ),    
    new ManualUp(elevator, 0.3).withTimeout(0.75),
    new BrakeEngage(elevator)
    );
  }
}
/* 
 * Previously, after Hartford (Sequential command group):
public ClimbAtEnd(Elevator elevator, AmpTrap ampTrap, Intake intake, Tilt tilt, Cartridge cartridge) {
addCommands( //assumes elevator starts at top
  new WaitCommand(0.5),
  new PIDDownToHeight(elevator, Constants.Elevator.JUST_ABOVE_CHAIN_HEIGHT).withTimeout(0.25),
  Commands.parallel(
      new PIDDownToHeight(elevator, Constants.Elevator.MIN_HEIGHT).withTimeout(3),
      new ClimbTrapShotWithWait(intake, cartridge, ampTrap, tilt).withTimeout(3)
      ),      
  new ClimbTrapShot(intake, cartridge, ampTrap, tilt).withTimeout(.75),
  new ManualUp(elevator, 0.3).withTimeout(0.75),
  new BrakeEngage(elevator)
    );
 * 
*/



