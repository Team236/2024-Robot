// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualIntakeWithWait extends SequentialCommandGroup {
  /** Creates a new ManualIntakeWithWait. */
  public ManualIntakeWithWait(Intake intake, double intSpeed) {
    addCommands(
       new WaitCommand(1),//1
       new ManualIntake(intake, intSpeed)
    ); 
  }
}


