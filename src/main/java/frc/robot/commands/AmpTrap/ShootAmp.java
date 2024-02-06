// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpTrap;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Cartridge.PIDCartridgeMotors;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAmp extends SequentialCommandGroup {
  /** Creates a new ShootAmp */
  //Shoots Amp - spins intake, cartridge (PID velocity), and Amp motors
  public ShootAmp(Intake intake, Cartridge cartridge, AmpTrap ampTrap, double intSpeed, double cartSpeed, double ampSpeed) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new ToStowedPosition(cartridge),
    Commands.parallel(
      new ManualIntake(intake, intSpeed),
      new PIDCartridgeMotors(cartridge, cartSpeed),
      new AmpMotor(ampTrap, ampSpeed)
      )
    );
  }
}
