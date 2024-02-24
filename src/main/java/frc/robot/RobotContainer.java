// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AmpTrap.AmpMotor;
import frc.robot.commands.AmpTrap.AmpShot;
import frc.robot.commands.Autos.AutoPIDDrive;
import frc.robot.commands.Autos.AutoPIDTurn;
import frc.robot.commands.Autos.FrontShootGrabShoot;
import frc.robot.commands.Autos.PIDPoseCommand;
import frc.robot.commands.Autos.WooferLeft;
import frc.robot.commands.CameraLimelight.AmpCameraAngle;
import frc.robot.commands.CameraLimelight.LLAngle;
import frc.robot.commands.CameraLimelight.LLDistance;
import frc.robot.commands.CameraLimelight.LLTarget;
import frc.robot.commands.CartridgeAndTilt.ManualExtCartridge;
import frc.robot.commands.CartridgeAndTilt.ManualPodiumSpeed;
import frc.robot.commands.CartridgeAndTilt.ManualRetractCartridge;
import frc.robot.commands.CartridgeAndTilt.ManualWooferSpeed;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeMotors;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeShot;
import frc.robot.commands.CartridgeAndTilt.PIDCartridgeTilt;
import frc.robot.commands.Drive.ArcadeXbox;
import frc.robot.commands.Drive.HighGear;
import frc.robot.commands.Drive.LowGear;
import frc.robot.commands.Drive.ToggleGear;
import frc.robot.commands.Elevator.ManualDown;
import frc.robot.commands.Elevator.ManualUp;
import frc.robot.commands.Elevator.PIDClimb;
import frc.robot.commands.Elevator.PIDDownToHeight;
import frc.robot.commands.Elevator.PIDUptoHeight;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.subsystems.AmpTrap;
import frc.robot.subsystems.Cartridge;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OdometryDrive;
import frc.robot.subsystems.Tilt;
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

  //create instance of each subsystem
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Cartridge cartridge = new Cartridge();
  private final AmpTrap ampTrap = new AmpTrap();
  private final Elevator elevator = new Elevator();
  private final Tilt tilt = new Tilt();
  private final OdometryDrive odometryDrive = new OdometryDrive();
  private Command command;

  //AUTO SWITCHES
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DriveConstants.DIO_AUTO_4);

  //create instance of each command
  //DRIVE COMMANDS
  private final ArcadeXbox arcadeXbox = new ArcadeXbox(drive.diffDrive, driverController, drive);
  private final LowGear lowGear = new LowGear(drive); 
  private final HighGear highGear = new HighGear(drive); 
  private final ToggleGear toggleGear = new ToggleGear(drive);

 //INTAKE COMMANDS:
  private final SetIntakeSpeed setIntakeSpeed = new SetIntakeSpeed(intake, Constants.Intake.INTAKE_SPEED);
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
  private final PIDCartridgeShot pidWooferShot = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.WOOFER_PID_RPM, true);
  private final PIDCartridgeShot pidPodiumShot = new PIDCartridgeShot(intake, cartridge, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.PODIUM_PID_RPM, false);
 
 //AMPTRAP COMMANDS:
  private final AmpMotor ampMotorForward = new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
  private final AmpMotor ampMotorReverse = new AmpMotor(ampTrap, Constants.Amp.AMP_TRAP_MOTOR_REVERSE_SPEED);
  private final AmpShot ampShot = new AmpShot(intake, cartridge, ampTrap, tilt, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.Amp.AMP_TRAP_MOTOR_SPEED);
  
 //AUTO COMMANDS
  private final AutoPIDDrive autoPIDDrive = new AutoPIDDrive(drive, Constants.DriveConstants.AUTO_DISTANCE_1);
  private final AutoPIDTurn autoPIDTurn = new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_1);
  private final AutoPIDTurn autoPIDTurn1 = new AutoPIDTurn(drive, Constants.DriveConstants.TURN_ANGLE_2);
  private final FrontShootGrabShoot frontShootGrabShoot = new FrontShootGrabShoot(intake, cartridge, tilt, drive, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM, Constants.DriveConstants.WOOFERFRONT_TO_NOTE);
  private final Transform2d transform  = new Transform2d();
  private final PIDPoseCommand pidPoseCommand = new PIDPoseCommand();

  //ELEVATOR COMMANDS:
  private final ManualUp manualUp = new ManualUp(elevator, Constants.Elevator.ELEV_UP_SPEED);
  private final ManualDown manualDown = new ManualDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);
  private final PIDUptoHeight pidToTop = new PIDUptoHeight(elevator, Constants.Elevator.TOP_HEIGHT);
  private final PIDDownToHeight pidToBot = new PIDDownToHeight(elevator, Constants.Elevator.BOTTOM_HEIGHT);
  private final PIDClimb climbPID = new PIDClimb(elevator, ampTrap, intake, tilt, cartridge);

  //CAMERA AND LIMELIGHT COMMANDS
private final LLAngle llAngle= new LLAngle(drive, 0);
private final LLDistance llDistance = new LLDistance(drive, 0, 60);  
private final LLTarget llTarget = new LLTarget(drive, 0, 40, 18);
private final AmpCameraAngle ampCameraAngle = new AmpCameraAngle(ampTrap);
private final AmpCameraAngle floorCameraAngle = new AmpCameraAngle(ampTrap);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // set only one default command for driving
    drive.setDefaultCommand(arcadeXbox);
    // drive.setDefaultCommand(tankXbox);
    // drive.setDefaultCommand(curvatureXbox);
    // drive.setDefaultCommand(tankJoysticks); //uses left Z to turn, not right X
    // drive.setDefaultCommand(arcadeJoysticks);
	
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
    //  Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //  new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //   m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // CREATE BUTTONS
    
    // *XBOXCONTROLLER - DRIVER CONTROLLER
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
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
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
	JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
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

//      assign button to comnands  
  //***** driver primary controller ******
  //A
    a.whileTrue(manualDown);
    //a.onTrue(podiumTilt);
    //a.onTrue(new AmpCameraAngle(ampTrapShooter));
  //B
      b.onTrue(pidToBot);
      //b.whileTrue(setIntakeSpeed);
      //b.onTrue(new FloorCameraAngle(ampTrapShooter));
  //X
      x.onTrue(pidToTop);
      //x.onTrue(toggleGear);
      //x.onTrue(stowTilt);
  //Y
      y.whileTrue(manualUp);
      //y.onTrue(wooferTilt);
  //other	
      //rb.onTrue(autoPIDDrive);
      //lb.onTrue(autoPIDTurn1);	
      view.onTrue(lowGear);
      menu.onTrue(highGear);	
  //POV
      upPov.whileTrue(llTarget);
    // upPov.onTrue(floorCameraAngle);
      leftPov.whileTrue(llAngle);
      downPov.whileTrue(llDistance);
      //downPov.onTrue(ampCameraAngle);

  //***** Aux Secondary Controller ******

  //A
    a1.whileTrue(manualPodiumSpeed);
  //B 
    b1.whileTrue(pidPodiumSpeed);//cartridge motors only
  //X
    x1.whileTrue(pidWooferSpeed);//cartridge motors only
    //x1.onTrue(pidWooferShot); //intake and cart motors, also tilt
    // x1.onTrue(frontShootGrabShoot);
  //Y
    y1.whileTrue(manualWooferSpeed);
    //y1.onTrue(pidPodiumShot); //intake and cart motors, also tilt
  //other	

  //POV
    upPov1.whileTrue(ampMotorForward);
    //upPov1.whileTrue(manualIntake);
    downPov1.whileTrue(ampMotorReverse);
    //downPov1.whileTrue(manualEject);
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
      command = pidPoseCommand;
    
    } else if (autoSwitch1.get() && !autoSwitch2.get() && autoSwitch3.get() && autoSwitch4.get()) {
    //command = new WooferLeft(intake, cartridge, tilt, drive, elevator, Constants.Intake.INTAKE_SPEED, Constants.CartridgeShooter.AMP_PID_RPM);
     command = frontShootGrabShoot;   //TODO replace this with working wooferleft
    } else if (autoSwitch1.get() && autoSwitch2.get() && !autoSwitch3.get() && autoSwitch4.get()) {
     command = frontShootGrabShoot;
     
    } else if (autoSwitch1.get() && autoSwitch2.get() && autoSwitch3.get() && !autoSwitch4.get()) {
      command = frontShootGrabShoot;
    }   
      return command;
    }

    	// Reset odometry to the initial pose of the trajectory, run path following
    	// command, then stop at the end.
    // return Commands.runOnce(() -> odometryDrive.resetOdometry(exampleTrajectory.getInitialPose()))
		//   .andThen(ramseteCommand)
		//   .andThen(Commands.runOnce(() -> odometryDrive.stop()));
		

 
}

