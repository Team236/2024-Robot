// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CameraLimelight.LLAngle;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.DriveCommands.LowGear;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LLAngleLowGear extends SequentialCommandGroup {
  /** Creates a new LLAngleLowGear. */
  public LLAngleLowGear(Drive drive, double pipeline) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
//new LowGear(drive),
      new LLAngle(drive, pipeline),
      new ToggleGear(drive)
    );
  }
}
