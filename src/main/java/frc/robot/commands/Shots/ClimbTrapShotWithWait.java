// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AmpTrap.AmpMotor;
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
public class ClimbTrapShotWithWait extends SequentialCommandGroup {
  /** Creates a new ClimbWithWait. */
  public ClimbTrapShotWithWait(Intake intake, Cartridge cartridge, AmpTrap ampTrap, Tilt tilt) {
    addCommands(
      Commands.parallel(
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(0.25),
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.AMP_PID_LEFT_RPM,  Constants.CartridgeShooter.AMP_PID_RIGHT_RPM).withTimeout(0.25),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(0.25) 
      ),
      Commands.parallel(         
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW).withTimeout(4), 
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.AMP_PID_LEFT_RPM, Constants.CartridgeShooter.AMP_PID_RIGHT_RPM).withTimeout(4),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(4),
      new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(4)
      )
    );
    Intake.resetCounter();
  }
}

/** JUST AFTER HARTFORD (SEQUENTIAL)
 *  Creates a new ClimbWithWait. 
  public ClimbTrapShotWithWait(Intake intake, Cartridge cartridge, AmpTrap ampTrap, Tilt tilt) {
    addCommands(
      new WaitCommand(0.5),
      Commands.parallel(      
      new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(4),
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.AMP_PID_RPM).withTimeout(4),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(4))
    );
    Intake.resetCounter();
  }
}
*/ 

/** 
 * 
 * SEQ PREVIOUSLY:
 *    Commands.parallel(
      new WaitCommand(0.5),
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.AMP_PID_RPM).withTimeout(4),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(4)
      ),
      Commands.parallel(          
      new ManualIntake(intake, Constants.Intake.INTAKE_SPEED).withTimeout(4),
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.AMP_PID_RPM).withTimeout(4),
      new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED).withTimeout(4))
*/