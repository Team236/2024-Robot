// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CartridgeAndTilt;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CameraLimelight.LLAngle;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LLPressandHold extends ParallelCommandGroup {
  /** Creates a new LLPressandHold. */
  public LLPressandHold(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, int pipeline) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PidLLTilt(tilt, pipeline), //.withTimeout(1),
      new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_RPM)
    );
  }
}
