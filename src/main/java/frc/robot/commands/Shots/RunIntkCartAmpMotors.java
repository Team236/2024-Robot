// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntkCartAmpMotors extends ParallelCommandGroup {
  /** Creates a new RunCartandAmpMotors. */
  public RunIntkCartAmpMotors(Intake intake, Cartridge cartridge, AmpTrap ampTrap, double intSpeed, double cartSpeedLeft, double cartSpeedRight, double ampSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ManualIntakeWithWait(intake, intSpeed),
      new AmpMotor(ampTrap, ampSpeed),
      new PIDCartridgeMotors(cartridge, cartSpeedLeft, cartSpeedRight)
      );
      Intake.resetCounter();  //reset counter after shooting a Note
  }
}

   