// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Compressor compressor;
  public AHRS navx = new AHRS();
  public UsbCamera usbCamera0;
  public static Servo cameraServo;
  public static boolean isFloor;


  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();
    
  //USB camera and servo:
   cameraServo = new Servo(Constants.PWM_FRONT_CAM);
   // default the camera to point at front floor as starting position

		try {
      usbCamera0 = CameraServer.startAutomaticCapture(0);
    } catch (Exception e)  {
        SmartDashboard.putString("camera capture failed", "failed");
    }  

      //usbCamera0.setResolution(320, 240);
      //usbCamera0.setBrightness(50);// percentage 0 to 100
      //usbCamera0.setExposureManual(50); //percentage 0 to 100

    //Need to do this once only in order to have Limelight communication while tethered
    for (int port = 5800; port <= 5805; port++){
      PortForwarder.add(port, "limelight.local", port);
    }

    LimelightHelpers.setLEDMode_ForceOff("limelight");

    // set the camera relative to the robot center point at floor
    /* Limelight Camera Space - 3d Cartesian Coordinate System with (0,0,0) at the camera lens.
      * X+ → Pointing to the right (if you were to embody the camera)
      * Y+ → Pointing downward
      * Z+ → Pointing out of the camera    
       Robot Space - 3d Cartesian Coordinate System with (0,0,0) located at the center of the robot’s frame projected down to the floor.
      * X+ → Pointing forward (Forward Vector)
      * Y+ → Pointing toward the robot’s right (Right Vector)
      * Z+ → Pointing upward (Up Vector)  
    */
    LimelightHelpers.setCameraPose_RobotSpace("limelight"
        , Units.inchesToMeters(14)    // x = 14
        , Units.inchesToMeters(0)     // y = 0
        , Units.inchesToMeters(43.5)  // z distance from floor to center of Limelight lens is 43.5
        , Units.degreesToRadians(0)  // roll = 0
        , Units.degreesToRadians(9.7)   // pitch =  9.7 degres //degrees to rads, camera tilt, up from horizontal
        , Units.degreesToRadians(0)   // yaw = 0
      );

    //Set the counter to zero at the start
    Intake.resetCounter();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("Match Time:",Timer.getMatchTime());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();



    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }



  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {    
      //Cartridge.setRPM_PID(2500);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  Robot.cameraServo.set(Constants.FRONT_CAM_TELEOP);
   isFloor = true; //start match with camera aimed at floor

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Match Time: ", Timer.getMatchTime());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
