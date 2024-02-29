// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.Autos.PIDDrive;
import frc.robot.commands.Autos.PIDTurn;
import frc.robot.commands.Autos.FrontTwoShots;
import frc.robot.commands.Autos.WooferLeft;
import frc.robot.commands.Autos.WooferRight;
import frc.robot.commands.CameraLimelight.CameraAngle;
import frc.robot.commands.CameraLimelight.LLTurn;
import frc.robot.commands.CameraLimelight.LLDistance;
import frc.robot.commands.CameraLimelight.LLTarget;
import frc.robot.commands.CameraLimelight.CameraToggle;
import frc.robot.commands.CameraLimelight.LLAngle;
import frc.robot.commands.CartridgeAndTilt.LLPressandHold;
import frc.robot.commands.CartridgeAndTilt.ManualExtCartridge;
import frc.robot.commands.CartridgeAndTilt.ManualPodiumSpeed;
import frc.robot.commands.CartridgeAndTilt.ManualRetractCartridge;
import frc.robot.commands.CartridgeAndTilt.ManualWooferSpeed;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.CartridgeAndTilt.PidLLTilt;
import frc.robot.commands.CartridgeAndTilt.ShootButtonPressandHold;
import frc.robot.commands.CartridgeAndTilt.ShootButtonRelease;
import frc.robot.commands.Drive.ArcadeXbox;
import frc.robot.commands.Drive.HighGear;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.BrakeEngage;
import frc.robot.commands.Elevator.ManualDown;
import frc.robot.commands.Elevator.ManualUp;
import frc.robot.commands.Elevator.ClimbPID;
import frc.robot.commands.Elevator.ClimbPIDWithManual;
import frc.robot.commands.Elevator.ClimbNoBrakePID;
import frc.robot.commands.Elevator.PIDDownToHeight;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Elevator.PIDtoTopandStow;
import frc.robot.commands.Elevator.BrakeToggle;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Shots.AmpShot;
import frc.robot.commands.Shots.PIDCartridgeShot;
import frc.robot.commands.Shots.PIDLLShot;
import frc.robot.commands.Shots.PIDPodShotWithBlueTurn;
import frc.robot.commands.Shots.PIDPodShotWithRedTurn;
import frc.robot.commands.Shots.RunIntkCartAmpMotors;
import frc.robot.commands.Shots.RunIntkCartMotors;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.AmpTrap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  //CONTROLLERS
  XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);
  //AUTO SWITCHES
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_4);

  //create instance of each subsystem
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Cartridge cartridge = new Cartridge();
  private final AmpTrap ampTrap = new AmpTrap();
  private final Elevator elevator = new Elevator();
  private final Tilt tilt = new Tilt();

  //create instance of each command
//DRIVE COMMANDS
  private final ArcadeXbox arcadeXbox = new ArcadeXbox(drive.diffDrive, driverController, drive);
  private final ToggleGear toggleGear = new ToggleGear(drive); 
  //private final PIDTurn pidTurn180 =  new PIDTurn(drive, 180); 
  //private final PIDDrive pidDrive = new PIDDrive(drive, Constants.DriveConstants.AUTO_DISTANCE_1);
  //private final PIDTurn pidTurnPodtoWoofRed = new PIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_RED_POD_TO_SPKR); 
  //private final PIDTurn pidTurnPodtoWoofBlue = new PIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_BLUE_POD_TO_SPKR);
//SHOTS
  private final AmpShot ampShot = new AmpShot(intake, cartridge, ampTrap, tilt);
  private final PIDCartridgeShot pidPodiumShot = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_PODIUM);
  private final PIDCartridgeShot pidWooferShot = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER);
  private final PIDLLShot pidLLShot = new PIDLLShot(intake, cartridge, tilt, drive, 0);
  private final PIDPodShotWithBlueTurn pidPodShotWithBlueTurn = new PIDPodShotWithBlueTurn(intake, cartridge, tilt, drive);
  private final PIDPodShotWithRedTurn pidPodShotWithRedTurn = new PIDPodShotWithRedTurn(intake, cartridge, tilt, drive);
//AUTO COMMANDS
  private final FrontTwoShots frontTwoShots = new FrontTwoShots(intake, cartridge, tilt, drive, elevator);
  private final WooferLeft wooferLeft = new WooferLeft(intake, cartridge, tilt, drive, elevator);
  private final WooferRight wooferRight = new WooferRight(intake, cartridge, tilt, drive, elevator);
//INTAKE COMMANDS
  private final IntakeWithCounter intakeWithCounter = new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED);
  private final ManualIntake manualIntake = new ManualIntake(intake, Constants.Intake.INTAKE_SPEED);
  private final ManualIntake manualEject = new ManualIntake(intake, Constants.Intake.EJECT_SPEED);
//CARTRIDGE AND TILT COMMANDS
  private final ManualExtCartridge manualExtCartridge = new ManualExtCartridge(tilt, Constants.Tilt.MAN_EXT_SPEED);
  private final ManualRetractCartridge manualRetCartridge = new ManualRetractCartridge(tilt, Constants.Tilt.MAN_RET_SPEED);
  private final PIDCartridgeTilt podiumTilt = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM);
  private final PIDCartridgeTilt wooferTilt = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER);
  private final PIDCartridgeTilt stowTilt = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW);
  private final PidLLTilt pidLLTilt = new PidLLTilt(tilt, 0);
  private final ShootButtonPressandHold shootPressAndHold = new ShootButtonPressandHold(intake, cartridge, tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER);
  private final ShootButtonRelease shootButtonRelease = new ShootButtonRelease(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM);
  //private final ManualPodiumSpeed manualPodiumSpeed = new ManualPodiumSpeed(cartridge);   Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER
  //private final ManualWooferSpeed manualWooferSpeed = new ManualWooferSpeed(cartridge);
  //private final PIDCartridgeMotors pidPodiumSpeed = new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.PODIUM_PID_RPM);
  //private final PIDCartridgeMotors pidWooferSpeed = new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_RPM);
  //private final RunIntkCartMotors wooferIntkCartMotors = new RunIntkCartMotors(intake, cartridge, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM);
  //private final RunIntkCartAmpMotors runIntCartAmpMotors = new RunIntkCartAmpMotors(intake, cartridge, ampTrap,  Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
//ELEVATOR COMMANDS:
  private final ManualUp manualUp = new ManualUp(elevator, Constants.Elevator.ELEV_UP_SPEED);
  private final ManualDown manualDown = new ManualDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);
  private final PIDtoTopandStow pidtoTopandStow = new PIDtoTopandStow(elevator, tilt);
  private final ClimbPID climbPID = new ClimbPID(elevator, ampTrap, intake, tilt, cartridge);
  private final ClimbNoBrakePID climbNoBrakePID = new ClimbNoBrakePID(elevator, ampTrap, intake, tilt, cartridge);
  private final BrakeToggle toggleBrake = new BrakeToggle(elevator);
  private final ClimbPIDWithManual climbPIDWithManual = new ClimbPIDWithManual(elevator, ampTrap, intake, tilt, cartridge);
  //private final PIDUptoHeight pidToTop = new PIDUptoHeight(elevator, Constants.Elevator.MAX_HEIGHT);
  //private final PIDDownToHeight pidToBot = new PIDDownToHeight(elevator, Constants.Elevator.MIN_HEIGHT);
  //private final BrakeEngage engageBrake = new BrakeEngage(elevator);
  //private final PIDUptoHeight pidUpToMatchHeight = new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT);
//CAMERA AND LIMELIGHT COMMANDS
  private final LLAngle llAngle = new LLAngle(drive);
  private final LLPressandHold llPressandHold = new LLPressandHold(intake, cartridge, tilt, 0);
  private final CameraToggle toggleCameraAngle = new CameraToggle();
  //private final LLDistance llDistance = new LLDistance(drive, 0, 60, 18);
  //private final LLTarget llTarget = new LLTarget(drive, 0, 40, 18);
  //private final CameraAngle ampCameraAngle = new CameraAngle(Constants.FRONT_CAM_TRAP);
  //private final CameraAngle floorCameraAngle = new CameraAngle(Constants.FRONT_CAM_TELEOP);
//AMPTRAP COMMANDS:
 // private final AmpMotor ampMotorForward = new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
 // private final AmpMotor ampMotorReverse = new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_REVERSE_SPEED);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(arcadeXbox);
    //drive.setDefaultCommand(tankXbox);  //drive.setDefaultCommand(curvatureXbox);
    //drive.setDefaultCommand(tankJoysticks); //drive.setDefaultCommand(arcadeJoysticks);

    // Configure the trigger bindings
    configureBindings();
  }
  private void configureBindings() {
    // CREATE BUTTONS
    // XBOXCONTROLLER - DRIVER CONTROLLER
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
    JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y);
    JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);
    JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);
    POVButton upPov = new POVButton(driverController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driverController,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driverController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driverController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    // XBOX CONTROLLER - AUX CONTROLLER
    JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);
    JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);
    JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE);
    POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);

  //assign button to commands
    
  //***** driver controller ******
  //INTAKE AND SHIFT
    rb.onTrue(toggleGear);   
    lb.onTrue(intakeWithCounter);
    rm.whileTrue(manualIntake); 
    lm.whileTrue(manualEject);
  //TURNS
    y.onTrue(llAngle);
  //CAMERA
    view.onTrue(toggleCameraAngle);


  //***** Aux Controller ******
  //SHOTS
    //a1.onTrue(pidWooferShot);
    a1.whileTrue(llPressandHold).onFalse(shootButtonRelease);
    b1.whileTrue(shootPressAndHold).onFalse(shootButtonRelease);
    y1.onTrue(ampShot);
    x1.onTrue(pidPodiumShot);
    leftPov1.onTrue(pidPodShotWithBlueTurn); 
    rightPov1.onTrue(pidPodShotWithRedTurn); 
  //ELEVATOR - zero manually before using PID
    upPov1.whileTrue(manualUp);
    downPov1.whileTrue(manualDown);
    lb1.onTrue(toggleBrake);
    rb1 .onTrue(pidtoTopandStow);
    view1.onTrue(climbNoBrakePID);
    //view1.onTrue(climbPIDWithManual);
    menu1.onTrue(climbPID);
  //CARTRIDGE TILT - zero before using PID
    lm1.whileTrue(manualRetCartridge);
    rm1.whileTrue(manualExtCartridge);


   // upPov.onTrue(frontTwoShots);
    //downPov.onTrue(wooferLeft);
    //leftPov.onTrue(wooferRight);
    //a.onTrue(pidTurn180);
    //b.onTrue(pidTurnPodtoWoofRed);
    //x.onTrue(pidTurnPodtoWoofBlue); 
    //view1.whileTrue(ampMotorReverse);
    //menu1.whileTrue(ampMotorForward);
    //view1.whileTrue(pidWooferSpeed);
    //menu1.whileTrue(pidPodiumSpeed);
    //lm.whileTrue(runIntCartAmpMotors); 
    //rm.whileTrue(wooferIntkCartMotors);
    //lm1.onTrue(lowGear);
    //rm1.onTrue(highGear);
    //lb1.onTrue(engageBrake);
    //lm.onTrue(pidDrive);
    //leftPov1.onTrue(pidUpToMatchHeight);
    //leftPov1.whileTrue(manualUp);
    //rightPov1.whileTrue(climbManualDown); 
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // SmartDashboard.putString("autokey", "Entering getAutoCommand now");
     Command command = null;
    //Swith 1 OFF, 2 ON, 3 ON, 4 ON
    if (!autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
      command = wooferLeft;
      //SmartDashboard.putNumber("Auto Switch is: ", 1);
    //Swith 1 ON, 2 OFF, 3 ON, 4 ON
    } else if (autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
      command = frontTwoShots;
        // SmartDashboard.putNumber("Auto Switch is: ", 2);
    //Swith 1 ON, 2 ON, 3 OFF, 4 ON
    } else if (autoSwitch1.get() && autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {
      command = frontTwoShots;
        // SmartDashboard.putNumber("Auto Switch is: ", 3);
      //Swith 1 ON, 2 ON, 3 OFF, 4 OFF
    } else if (autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && !autoSwitch4.get()) {
      command =  wooferRight;
        // SmartDashboard.putNumber("Auto Switch is: ", 4);
   }
   return command;
  }

}