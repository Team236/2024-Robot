// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  public UsbCamera usbRearCam;
  public UsbCamera usbFrontCam;
  public static Servo frontCamServo;
  NetworkTableEntry cameraSelection;

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

  //Cameras- PERIODIC CODE SWITCHES TO REAR CAMERA VIEW WHEN MENU1 IS PRESSED
    frontCamServo = new Servo(Constants.PWM_FRONT_CAM);
  try {
    usbFrontCam = CameraServer.startAutomaticCapture(0);
    usbRearCam = CameraServer.startAutomaticCapture(1);
} catch (Exception e)  {
    SmartDashboard.putString("camera capture failed", "failed");
  }

  cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

  //if using Limelight:  don't use driver mode, rather create "driver" pipeline, reduce exposure,
  //use the "stream" NT key to enable picture in picture mode (USB cam / LL) - big reduction in bandwidth
  //hardware settings: 320x240 Res, MJPEG format, 15-20 FPS (reduce as needed to cpu usage)
  //stream settings: 320x240 Res, MJPEG, 10-15 FPS, compression 30 (adjust as needed to get desired cpu usage)
  //TODO:  how to set stream settings?

  //TODO - BE SURE TO SET DASHBOARD RESOLUTION SAME AS THIS
    usbFrontCam.setResolution(320, 240);
    usbRearCam.setResolution(320,240);
    usbFrontCam.setBrightness(60);// 0 to 100,  percentage
    usbRearCam.setBrightness(60);// 0 to 100,  percentage
    usbFrontCam.setExposureManual(50);// 0 to 100, as a percentage
    usbRearCam.setExposureManual(50);// 0 to 100, as a percentage
      // usbFrontCam.setFPS(15);
   // usbRearCam.setFPS(15);
    SmartDashboard.putNumber("FrontCam FPS: ", usbFrontCam.getActualFPS());
    SmartDashboard.putNumber("RearCam FPS: ", usbFrontCam.getActualFPS());
    
    // set camera position in robot relative to center on floor in meters 
    LimelightHelpers.setCameraPose_RobotSpace("limelight",
        Constants.Camera.LIMELIGHT_FWRD , Constants.Camera.LIMELIGHT_SIDE , Constants.Camera.LIMELIGHT_UP, 
        Constants.Camera.LIMELIGHT_ROLL, Constants.Camera.LIMELIGHT_PITCH, Constants.Camera.LIMELIGHT_YAW );
    // publish field location relative to _____ alience based on AprilTags    
    SmartDashboard.putNumberArray("limelightRobotPose",LimelightHelpers.getBotPose("limelight"));
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
    SmartDashboard.putNumberArray("limelightRobotPose",LimelightHelpers.getBotPose("limelight"));
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
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Selects FrontCamera when menu1 not pressed; RearCamera when menu1 pressed
     XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);
     JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    if (menu1.getAsBoolean()) {
      SmartDashboard.putBoolean("Switching camera to Rear Camera: ", true);
      cameraSelection.setString(usbRearCam.getName());
    } else {
       SmartDashboard.putBoolean("Using Front Camera: ", true);
        cameraSelection.setString(usbFrontCam.getName());
    }
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
