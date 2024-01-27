// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cartridge;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDCartridgeShot extends ParallelCommandGroup {
  /** Creates a new PIDCartridgeShot. */
  public PIDCartridgeShot(Intake intake, Cartridge cartridge, double intSpeed, double cartSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ManualIntake(intake, intSpeed).withTimeout(2),
      new PIDCartridgeMotors(cartridge, cartSpeed).withTimeout(2));
  }
}
