// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraLimelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.HighGear;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.subsystems.Drive;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LLAngle extends SequentialCommandGroup {
  /** Creates a new LLAngle. */
  public LLAngle(Drive drive) {
    addCommands(
      //new LowGear(drive),
      new LLTurn(drive, 0).withTimeout(1)//TODO - adjust timeout 
     // ,new HighGear(drive)
    );
  }
}
