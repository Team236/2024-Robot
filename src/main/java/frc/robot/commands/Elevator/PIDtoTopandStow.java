// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CameraLimelight.CameraAngle;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

public class PIDtoTopandStow extends ParallelCommandGroup {
//preparation for climb - elevator to top, cartridge stowed, camera angled at Amp
  public PIDtoTopandStow(Elevator elevator, Tilt tilt) {
    addCommands(
      new PIDUptoHeight(elevator, Constants.Elevator.MAX_HEIGHT),
      new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW),
      new CameraAngle(Constants.FRONT_CAM_TRAP)
    );
  }
}

