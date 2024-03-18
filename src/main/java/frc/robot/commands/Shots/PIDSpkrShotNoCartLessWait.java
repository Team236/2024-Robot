// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.ManualIntakeShortWait;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDSpkrShotNoCartLessWait extends ParallelCommandGroup {
  /** Creates a new PIDSpkrShotNoCart. */
  public PIDSpkrShotNoCartLessWait(Intake intake, Tilt tilt, double intSpeed, double desiredRevs) {
    addCommands(
    new PIDCartridgeTilt(tilt,desiredRevs).withTimeout(2.5),    
    new ManualIntakeShortWait(intake, intSpeed).withTimeout(2.5) //use manualIntake since counter =1 here
    );
  Intake.resetCounter(); 
  }
}
   