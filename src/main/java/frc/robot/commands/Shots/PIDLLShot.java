// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shots;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CameraLimelight.LLAngle;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.CartridgeAndTilt.PidLLTilt;
import frc.robot.commands.Intake.ManualIntakeWithWait;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDLLShot extends ParallelCommandGroup {
  /** Creates a new PIDCartShotLimeLight. */
  public PIDLLShot(Intake intake, Cartridge cartridge, Tilt tilt, Drive drive, int pipeline) {
      addCommands(
          new PidLLTilt(tilt, pipeline).withTimeout(4),
          new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.PODIUM_PID_LEFT_RPM, Constants.CartridgeShooter.PODIUM_PID_RIGHT_RPM).withTimeout(4),
          new ManualIntakeWithWait(intake, Constants.Intake.INTAKE_SPEED).withTimeout(4)//use manualIntake since counter =1 here
        );
    Intake.resetCounter();//reset counter after shooting a Note
}
}


