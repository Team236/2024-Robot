// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShotNoCartMotors extends ParallelCommandGroup {
  /** Creates a new AmpShotNoCartMotors. */
  public AmpShotNoCartMotors(Intake intake, AmpTrap ampTrap, Tilt tilt) {
    addCommands(
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(2.5),
      new ManualIntakeWithWait(intake, Constants.Intake.INTAKE_SPEED).withTimeout(2.5),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(2.5)
    );
    Intake.resetCounter();  //reset counter after shooting a Note
  }
}
