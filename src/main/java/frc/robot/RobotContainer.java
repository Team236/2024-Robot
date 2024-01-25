// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.ShootTrapAmp;
import frc.robot.commands.DriveCommands.ArcadeJoysticks;
import frc.robot.commands.DriveCommands.ArcadeXbox;
import frc.robot.commands.DriveCommands.CurvatureXbox;
import frc.robot.commands.DriveCommands.HighGear;
import frc.robot.commands.DriveCommands.LowGear;
import frc.robot.commands.DriveCommands.TankJoysticks;
import frc.robot.commands.DriveCommands.TankXbox;
import frc.robot.commands.DriveCommands.ToggleGear;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TrapAmpShooter;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private final TrapAmpShooter trapAmpShooter = new TrapAmpShooter();
 // private final DifferentialDrive diffDrive = new DifferentialDrive(null, null)

  //create instance of each command
  private final ArcadeXbox arcadeXbox = new ArcadeXbox(drive.diffDrive, driverController, drive);
 // private final TankXbox tankXbox = new TankXbox(drive.diffDrive, driverController, drive);
 // private final CurvatureXbox curvatureXbox = new CurvatureXbox(drive.diffDrive, driverController, drive);
 // private final ArcadeJoysticks arcadeJoysticks = new ArcadeJoysticks(drive.diffDrive, leftStick, rightStick, drive);
 // private final TankJoysticks tankJoysticks = new TankJoysticks(drive.diffDrive, leftStick, rightStick, drive);
 private final LowGear lowGear = new LowGear(drive); 
 private final HighGear highGear = new HighGear(drive); 
 private final ToggleGear toggleGear = new ToggleGear(drive); 
 private final SetIntakeSpeed setIntakeSpeed = new SetIntakeSpeed(intake, Constants.Intake.INTAKE_SPEED);
 private final ShootTrapAmp shootTrapAmp = new ShootTrapAmp(trapAmpShooter);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
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

    //assign button to comnands


    //***** driver controller ******
    view.onTrue(lowGear);
    menu.onTrue(highGear);
    x.onTrue(toggleGear);
    b.whileTrue(setIntakeSpeed);

    //***** Aux Controller ******
    a1.onTrue(shootTrapAmp.withTimeout(2));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
