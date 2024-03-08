// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootButtonRelease extends ParallelCommandGroup {
  /** Creates a new CartridgeButtonRelease. */
  public ShootButtonRelease(Intake intake, Cartridge cartridge, Tilt tilt, double intSpeed, double cartSpeedLeft, double cartSpeedRight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ManualIntake(intake, intSpeed).withTimeout(2.3), //use manualIntake since counter =1 here
      new PIDCartridgeMotors(cartridge, cartSpeedLeft, cartSpeedRight).withTimeout(2.3)
    );
  }
}
