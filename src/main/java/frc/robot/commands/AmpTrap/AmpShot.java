// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AmpTrap;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShot extends SequentialCommandGroup {
  //Shoots Amp - spins intake, cartridge (PID velocity), and Amp motors
  public AmpShot(Intake intake, Cartridge cartridge, AmpTrap ampTrap, Tilt tilt, double intSpeed, double cartSpeed, double ampSpeed) {
    addCommands(
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW),
    Commands.parallel(
      new ManualIntake(intake, intSpeed),
      new PIDCartridgeMotors(cartridge, cartSpeed),
      new AmpMotor(ampTrap, ampSpeed)
      )
    );
    Intake.resetCounter();  //reset counter after shooting a Note
  }
}
