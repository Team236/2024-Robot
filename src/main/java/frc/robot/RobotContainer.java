// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AmpTrap.ShootAmp;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.Autos.AutoPIDDrive;
import frc.robot.commands.Autos.AutoPIDTurn;
import frc.robot.commands.Autos.FrontShootGrabShoot;
import frc.robot.commands.Cartridge.PIDCartridgeMotors;
import frc.robot.commands.Cartridge.PIDCartridgeShot;
import frc.robot.commands.Cartridge.PIDCartridgeTilt;
import frc.robot.commands.Cartridge.ManualExtCartridge;
import frc.robot.commands.Cartridge.ManualPodiumCartOnly;
import frc.robot.commands.Cartridge.ManualRetractCartridge;
import frc.robot.commands.Cartridge.ManualWooferCartOnly;
import frc.robot.commands.Drive.ArcadeJoysticks;
import frc.robot.commands.Drive.ArcadeXbox;
import frc.robot.commands.Drive.CurvatureXbox;
import frc.robot.commands.Drive.HighGear;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.TankJoysticks;
import frc.robot.commands.Drive.TankXbox;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.SetElevatorHeight;
import frc.robot.commands.Elevator.ClimbPID;
import frc.robot.commands.Elevator.ManualDown;
import frc.robot.commands.Elevator.ManualUp;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.AmpTrap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // sticks/controllers
  XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);
 // Joystick leftStick = new Joystick(Constants.USB_LEFT_JOYSTICK);
 // Joystick rightStick = new Joystick(Constants.USB_RIGHT_JOYSTICK);
  //XboxController auxController = new XboxController(Constants.USB_AUXCONTROLLER);

  //create instance of each subsystem
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Cartridge cartridgeShooter = new Cartridge();
  private final AmpTrap ampTrapShooter = new AmpTrap();
  private final Elevator elevator = new Elevator();
  private final Tilt tilt = new Tilt();
 
  //create instance of each command
  //DRIVE COMMANDS
  private final ArcadeXbox arcadeXbox = new ArcadeXbox(drive.diffDrive, driverController, drive);
 // private final TankXbox tankXbox = new TankXbox(drive.diffDrive, driverController, drive);
 // private final CurvatureXbox curvatureXbox = new CurvatureXbox(drive.diffDrive, driverController, drive);
 // private final ArcadeJoysticks arcadeJoysticks = new ArcadeJoysticks(drive.diffDrive, leftStick, rightStick, drive);
 // private final TankJoysticks tankJoysticks = new TankJoysticks(drive.diffDrive, leftStick, rightStick, drive);
  private final LowGear lowGear = new LowGear(drive); 
  private final HighGear highGear = new HighGear(drive); 
  private final ToggleGear toggleGear = new ToggleGear(drive); 

 //INTAKE COMMANDS:
  private final SetIntakeSpeed setIntakeSpeed = new SetIntakeSpeed(intake, Constants.Intake.INTAKE_SPEED);
  private final ManualIntake manualIntake = new ManualIntake(intake, Constants.Intake.INTAKE_SPEED);
  private final ManualIntake manualEject = new ManualIntake(intake, Constants.Intake.EJECT_SPEED);

 //CARTRIDGE COMMANDS:
  private final ManualPodiumCartOnly speakerShotFromPodium = new ManualPodiumCartOnly(cartridgeShooter);
  private final ManualWooferCartOnly speakerShotFromWoofer = new ManualWooferCartOnly(cartridgeShooter);
  private final PIDCartridgeMotors pidPodiumShot = new PIDCartridgeMotors(cartridgeShooter, Constants.CartridgeShooter.PODIUM_PID_RPM);
  private final PIDCartridgeMotors pidWooferShot = new PIDCartridgeMotors(cartridgeShooter, Constants.CartridgeShooter.WOOFER_PID_RPM);
  private final PIDCartridgeShot pidActualWoofer = new PIDCartridgeShot(intake, cartridgeShooter, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, true);
  private final PIDCartridgeShot pidActualPodium = new PIDCartridgeShot(intake, cartridgeShooter, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, false);

  private final PIDCartridgeTilt toPodiumPosition = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_PODIUM, 
                 Constants.Tilt.KP_TILT,  Constants.Tilt.KI_TILT,  Constants.Tilt.KD_TILT);
  private final PIDCartridgeTilt toWooferPosition = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_WOOFER, 
                 Constants.Tilt.KP_TILT,  Constants.Tilt.KI_TILT,  Constants.Tilt.KD_TILT);
  private final PIDCartridgeTilt toStowPosition = new PIDCartridgeTilt(tilt, Constants.Tilt.TILT_ENC_REVS_STOW, 
                Constants.Tilt.KP_TILT,  Constants.Tilt.KI_TILT,  Constants.Tilt.KD_TILT);
 private final ManualRetractCartridge manualRetract = new ManualRetractCartridge(tilt, Constants.Tilt.MAN_RET_SPEED);
 private final ManualExtCartridge manualExtend = new ManualExtCartridge(tilt, Constants.Tilt.MAN_EXT_SPEED);
  
 //AMPTRAP COMMANDS:
  private final AmpMotor ampMotorForward = new AmpMotor(ampTrapShooter, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
  private final AmpMotor ampMotorReverse = new AmpMotor(ampTrapShooter, Constants.Amp.AMP_TRAP_MOTOR_REVERSE_SPEED);
  private final ShootAmp shootAmpMotor = new ShootAmp(intake, cartridgeShooter, ampTrapShooter, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
  
 //AUTO COMMANDS
  private final AutoPIDDrive autoPIDDrive = new AutoPIDDrive(drive, Constants.DriveConstants.AUTO_DISTANCE_1);
  private final AutoPIDTurn autoPIDTurn = new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_1);
  private final AutoPIDTurn autoPIDTurn1 = new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_2);
  private final FrontShootGrabShoot frontShootGrabShoot = new FrontShootGrabShoot(intake, cartridgeShooter, tilt, drive, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.DriveConstants.WOOFERFRONT_TO_NOTE);

  //ELEVATOR COMMANDS:
  private final ManualUp manualUp = new ManualUp(elevator, Constants.Elevator.ELEV_UP_SPEED);
  private final ManualDown manualDown = new ManualDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);
  private final SetElevatorHeight pidToTop = new SetElevatorHeight(elevator, Constants.Elevator.TOP_HEIGHT, Constants.Elevator.KP_ELEV_UP, Constants.Elevator.KI_ELEV_UP, Constants.Elevator.KD_ELEV_UP);
  private final SetElevatorHeight pidToBot = new SetElevatorHeight(elevator, Constants.Elevator.BOTTOM_HEIGHT, Constants.Elevator.KP_ELEV_DOWN, Constants.Elevator.KI_ELEV_DOWN, Constants.Elevator.KD_ELEV_DOWN);
  private final ClimbPID climbPID = new ClimbPID(elevator, ampTrapShooter, intake, cartridgeShooter);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(arcadeXbox);

    // Configure the trigger bindings
    configureBindings();
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // CREATE BUTTONS
    // *XBOXCONTROLLER - DRIVER CONTROLLER
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

    //Assign button to comnands
    
    //***** driver controller ******
    //view.onTrue(lowGear);
   // menu.onTrue(highGear);
    //x.onTrue(toggleGear);
    //b.whileTrue(setIntakeSpeed);
    // a.onTrue(elevatorDownPID);
    // y.onTrue(elevatorUpPID);
   // x.onTrue(pidToTop);
   // b.onTrue(pidToBot);
    //rb.onTrue(climbPID);
    //rb.onTrue(autoPIDDrive);
    //lb.onTrue(autoPIDTurn1);
    upPov.whileTrue(manualIntake);
    downPov.whileTrue(manualEject);

    b.whileTrue(ampMotorForward);
    x.onTrue(ampMotorReverse.withTimeout(5));
    y.onTrue(shootAmpMotor.withTimeout(5));

    //***** Aux Controller ******
   //x1.whileTrue(manualExtend);
   //b1.whileTrue(manualRetract);
   // y1.onTrue(toWooferPosition);
    //a1.onTrue(toStowPosition);
 
    //y1.whileTrue(manualUp);
    //a1.whileTrue(manualDown);
  a1.onTrue(toPodiumPosition);
   y1.onTrue(toStowPosition);
   b1.onTrue(toWooferPosition);
  
  // b1.onTrue(speakerShotFromPodium.withTimeout(2));
  //x1.onTrue(speakerShotFromWoofer.withTimeout(2));
  //y1.whileTrue(pidPodiumShot);
  //a1.whileTrue(pidWooferShot);
   //b1.onTrue(pidActualWoofer.withTimeout(2));
  //x1.onTrue(pidActualPodium);
  //x1.onTrue(frontShootGrabShoot);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
* 
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
