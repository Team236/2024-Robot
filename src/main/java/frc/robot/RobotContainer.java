// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.Autos.AutoPIDDrive;
import frc.robot.commands.Autos.AutoPIDTurn;
import frc.robot.commands.Autos.FrontShootGrabShoot;
import frc.robot.commands.Autos.WooferLeft;
import frc.robot.commands.Autos.WooferRight;
import frc.robot.commands.CameraLimelight.CameraAngle;
import frc.robot.commands.CameraLimelight.LLAngle;
import frc.robot.commands.CameraLimelight.LLDistance;
import frc.robot.commands.CameraLimelight.LLTarget;
import frc.robot.commands.CameraLimelight.ToggleCameraAngle;
import frc.robot.commands.CartridgeAndTilt.ManualExtCartridge;
import frc.robot.commands.CartridgeAndTilt.ManualPodiumSpeed;
import frc.robot.commands.CartridgeAndTilt.ManualRetractCartridge;
import frc.robot.commands.CartridgeAndTilt.ManualWooferSpeed;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.ArcadeXbox;
import frc.robot.commands.Drive.HighGear;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.ManualDown;
import frc.robot.commands.Elevator.ManualUp;
import frc.robot.commands.Elevator.PIDActualClimb;
import frc.robot.commands.Elevator.PIDDownToHeight;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.IntakeWithCounter;
import frc.robot.commands.Shots.AmpShot;
import frc.robot.commands.Shots.PIDCartridgeShot;
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
  // The robot's subsystems and commands are defined here...sticks/controllers
  XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

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
  private final LowGear lowGear = new LowGear(drive); 
  private final HighGear highGear = new HighGear(drive); 
  private final ToggleGear toggleGear = new ToggleGear(drive); 
//SHOTS
 private final AmpShot ampShot = new AmpShot(intake, cartridge, ampTrap, tilt);
 private final PIDCartridgeShot pidPodiumShot = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_PODIUM);
 private final PIDCartridgeShot pidWooferShot = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, Constants.Tilt.TILT_ENC_REVS_WOOFER);
 private final PIDCartridgeShot pidFrontAutoShot2 = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, Constants.Tilt.TILT_ENC_REVS_AUTOSHOT2);
 private final RunIntkCartMotors wooferIntkCartMotors = new RunIntkCartMotors(intake, cartridge, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM);
 private final RunIntkCartAmpMotors runIntCartAmpMotors = new RunIntkCartAmpMotors(intake, cartridge, ampTrap,  Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
//INTAKE COMMANDS:
  private final IntakeWithCounter intakeWithCounter = new IntakeWithCounter(intake, Constants.Intake.INTAKE_SPEED);
  private final ManualIntake manualIntake = new ManualIntake(intake, Constants.Intake.INTAKE_SPEED);
  private final ManualIntake manualEject = new ManualIntake(intake, Constants.Intake.EJECT_SPEED);
//CARTRIDGE AND TILT COMMANDS:
  private final ManualExtCartridge manualExtCartridge = new ManualExtCartridge(tilt, Constants.Tilt.MAN_EXT_SPEED);
  private final ManualRetractCartridge manualRetCartridge = new ManualRetractCartridge(tilt, Constants.Tilt.MAN_RET_SPEED);
  private final PIDCartridgeTilt podiumTilt = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM);
  private final PIDCartridgeTilt wooferTilt = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER);
  private final PIDCartridgeTilt stowTilt = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW);

  private final ManualPodiumSpeed manualPodiumSpeed = new ManualPodiumSpeed(cartridge);
  private final ManualWooferSpeed manualWooferSpeed = new ManualWooferSpeed(cartridge);
  private final PIDCartridgeMotors pidPodiumSpeed = new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.PODIUM_PID_RPM);
  private final PIDCartridgeMotors pidWooferSpeed = new PIDCartridgeMotors(cartridge, Constants.CartridgeShooter.WOOFER_PID_RPM);
//AMPTRAP COMMANDS:
  private final AmpMotor ampMotorForward = new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
  private final AmpMotor ampMotorReverse = new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_REVERSE_SPEED);
//AUTO COMMANDS
  private final AutoPIDDrive autoPIDDrive = new AutoPIDDrive(drive, Constants.DriveConstants.AUTO_DISTANCE_1);
  private final AutoPIDTurn autoPIDTurn = new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_1); //90
  private final AutoPIDTurn autoPIDTurn1 = new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_2); //-90
  private final FrontShootGrabShoot frontShootGrabShoot = new FrontShootGrabShoot(intake, cartridge, tilt, drive, elevator);
  private final WooferLeft wooferLeft = new WooferLeft(intake, cartridge, tilt, drive, elevator);
  private final WooferRight wooferRight = new WooferRight(intake, cartridge, tilt, drive, elevator);
//AUTO SWITCHES
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_4);
//ELEVATOR COMMANDS:
  private final ManualUp manualUp = new ManualUp(elevator, Constants.Elevator.ELEV_UP_SPEED);
  private final ManualDown manualDown = new ManualDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);
   private final ManualDown climbManualDown = new ManualDown(elevator, Constants.Elevator.ELEV_MAN_DOWN_SPEED);
  private final PIDUptoHeight pidToTop = new PIDUptoHeight(elevator, Constants.Elevator.MAX_HEIGHT);
  private final PIDDownToHeight pidToBot = new PIDDownToHeight(elevator, Constants.Elevator.MIN_HEIGHT);
  private final PIDUptoHeight pidUpToMatchHeight = new PIDUptoHeight(elevator, Constants.Elevator.MATCH_HEIGHT);
  private final PIDActualClimb climbPID = new PIDActualClimb(elevator, ampTrap, intake, tilt, cartridge);
//CAMERA AND LIMELIGHT COMMANDS
  private final LLAngle llAngle= new LLAngle(drive, 0);
  private final LLDistance llDistance = new LLDistance(drive, 0, 60, 18);
  private final LLTarget llTarget = new LLTarget(drive, 0, 40, 18);
  private final CameraAngle ampCameraAngle = new CameraAngle(Constants.FRONT_CAM_AMP);
  private final CameraAngle floorCameraAngle = new CameraAngle(Constants.FRONT_CAM_FLOOR);
  private final ToggleCameraAngle toggleCameraAngle = new ToggleCameraAngle();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(arcadeXbox);
    //drive.setDefaultCommand(tankXbox);
    //drive.setDefaultCommand(curvatureXbox);
    //drive.setDefaultCommand(tankJoysticks); //uses left Z to turn, not right X
    //drive.setDefaultCommand(arcadeJoysticks);

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

    //assign button to comnands
    
    //***** driver controller ******
    //INTAKE
    rb.whileTrue(manualIntake);   
    rm.whileTrue(manualEject);
    a.whileTrue(intakeWithCounter); //USE THIS FOR MATCH
    //TILT- zero at the retracted limit before using PID!!!
    x.whileTrue(manualRetCartridge);
    b.whileTrue(manualExtCartridge);
    y.onTrue(stowTilt); //PID
    menu.onTrue(podiumTilt); //PID
    view.onTrue(wooferTilt); //PID
   //SHOT COMMAND GROUPS
    upPov.whileTrue(runIntCartAmpMotors); //runs all shot motors, cart motors set to AMP_PID_RPM
    downPov.whileTrue(wooferIntkCartMotors); //runs intake and cart motors, cart motors set to WOOFER_PID_RPM
    leftPov.onTrue(pidPodiumShot);
    rightPov.onTrue(pidWooferShot);
    lb.onTrue(pidFrontAutoShot2);
    lm.onTrue(ampShot);
    //lm.whileTrue(llAngle);

    //***** Aux Controller ******
    //CARTRIDGE MOTORS
    //a1.whileTrue(pidPodiumSpeed);
    a1.onTrue(frontShootGrabShoot);
    //b1.onTrue(wooderLeft);
    b1.onTrue(toggleCameraAngle);
    x1.onTrue(ampCameraAngle);
    y1.onTrue(floorCameraAngle);
    view1.whileTrue(pidWooferSpeed);
    menu1.whileTrue(pidPodiumSpeed);
    //AMP
    //view1.whileTrue(ampMotorReverse);
    //menu1.whileTrue(ampMotorForward);
    
    //ELEVATOR - zero at the lower limit before using PID!!!
    upPov1.onTrue(pidToTop);
    downPov1.onTrue(pidToBot);
    leftPov1.onTrue(pidUpToMatchHeight); //do this going up
    //leftPov1.whileTrue(climbManualDown);//for climb with constant speed 0.8
    rightPov1.onTrue(climbPID);
    lb1.whileTrue(manualUp);
    rb1.whileTrue(manualDown);

    //DRIVE
    //b1.onTrue(lowGear);
    //x1.onTrue(highGear);
   lm1.onTrue(toggleGear);
    //a1.onTrue(frontShootGrabShoot);
    //view1.whileTrue(wooferLeft);
    //lm1.onTrue(autoPIDDrive);
    //rm1.onTrue(autoPIDTurn1);
    //downPov1.whileTrue(llDistance);
    //upPov1.whileTrue(llTarget);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        SmartDashboard.putBoolean("Autoswitch1: ", !autoSwitch1.get());
        SmartDashboard.putBoolean("Autoswitch2: ", !autoSwitch2.get());
        SmartDashboard.putBoolean("Autoswitch3: ", !autoSwitch3.get());
        SmartDashboard.putBoolean("Autoswitch4: ", !autoSwitch4.get());
    if (!autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
      return wooferLeft;
     //return (new WooferLeft(intake, cartridge, tilt, drive, elevator, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM));
    } else if (autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
      return wooferRight;
    //return (new WooferRight(intake, cartridge, tilt, drive, elevator, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM));
    } else if (autoSwitch1.get() && autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {
     return frontShootGrabShoot;
     //return (new FrontShootGrabShoot(intake, cartridge, tilt, drive, elevator, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.DriveConstants.WOOFERFRONT_TO_NOTE));} 
    } else if (autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && !autoSwitch4.get()) {
      return frontShootGrabShoot;
      //  return (new FrontShootGrabShoot(intake, cartridge, tilt, drive, elevator, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.DriveConstants.WOOFERFRONT_TO_NOTE));
    } else {
      return null;
      }
    }
  }
