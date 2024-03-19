// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeShortWait;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShot extends SequentialCommandGroup
 {
  //Shoots Amp - 
  //First tilts cartridge to stow; then spins intake, cartridge (PID velocity), and Amp motors
  public AmpShot(Intake intake, Cartridge cartridge, AmpTrap ampTrap, Tilt tilt) {
    addCommands(
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(.4),
    Commands.parallel(
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.AMP_PID_LEFT_RPM, Constants.CartridgeShooter.AMP_PID_RIGHT_RPM).withTimeout(4),
     // new ManualIntakeShortWait(intake, Constants.Intake.INTAKE_SPEED).withTimeout(4),
      new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(4),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(4)
      )
    );
    Intake.resetCounter();  //reset counter after shooting a Note
  }
}
