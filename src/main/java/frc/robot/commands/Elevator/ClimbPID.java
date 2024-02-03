// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AmpTrap.WaitShootTrap;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbPID extends SequentialCommandGroup {
  /** Creates a new ClimbPID. */

  public ClimbPID(Elevator elevator, AmpTrap ampTrap, Intake intake, Cartridge cartridge) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorHeight(elevator, Constants.Elevator.TOP_HEIGHT, Constants.Elevator.KP_ELEV_UP, Constants.Elevator.KI_ELEV_UP, Constants.Elevator.KD_ELEV_UP).withTimeout(5),
      Commands.parallel(
      new SetElevatorHeight(elevator, Constants.Elevator.BOTTOM_HEIGHT, Constants.Elevator.KP_ELEV_CLIMB, Constants.Elevator.KI_ELEV_CLIMB, Constants.Elevator.KD_ELEV_CLIMB),
      new WaitShootTrap(intake, cartridge, ampTrap, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.MANUAL_SET_SPEED, Constants.Amp.AMP_TRAP_MOTOR_SPEED)
      )
    );
    
    
  }
}
