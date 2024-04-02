
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
public final class Constants {
  //camera servo motor, created in Robot.java:
  public static final int PWM_FRONT_CAM = 0;
  //for aiming camera at Amp/Trap or at Floor (from 0 to 1, for 0 to 180 degrees)
  public static final double FRONT_CAM_TRAP = 0.1;  //TODO adjust these as needed
  public static final double FRONT_CAM_TELEOP = 0.3; //TODO adjust as needed

  public static class Controller {
    // usb port on the laptop when driver using XBox controller
    public static final int USB_DRIVECONTROLLER = 0;//for driver
    public static final int USB_AUXCONTROLLER = 1; // for controller operator
    // usb ports on the laptop, when driver using two Thrustmaster Joysticks
    // public static final int USB_LEFT_JOYSTICK = 1;
    // public static final int USB_RIGHT_JOYSTICK = 2;
  }
  // ID numbers for the motor controllers:
  public static class MotorControllers {
    public static final int ID_LEFT_FRONT = 40; 
    public static final int ID_LEFT_REAR = 41;  
    public static final int ID_RIGHT_FRONT = 42;
    public static final int ID_RIGHT_REAR = 43;
    //Intake and Amp
    public static final int ID_INTAKE = 48;
    public static final int ID_AMP_TRAP_SHOOTER = 49;
    // Elevator
    public static final int ID_ELEVATOR_LEFT = 46;
    public static final int ID_ELEVATOR_RIGHT = 47;
    // Cartridge shooter
    public static final int ID_CARTRIDGE_LEFT = 44; 
    public static final int ID_CARTRIDGE_RIGHT = 45;

    public static final int ID_CARTRIDGE_TILT = 50; 

    public static final int SMART_CURRENT_LIMIT = 40;
    //Ramp rates (time in seconds to go from zero to full throttle)
    public static final double CLOSED_RAMP_RATE = 0.08;
    public static final double OPEN_RAMP_RATE = 0.08;
      }
  public static class Amp{
    public static final double AMP_TRAP_MOTOR_SPEED = 1.0; 
    public static final double AMP_TRAP_MOTOR_REVERSE_SPEED =-0.5; 
    }
  public static class DriveConstants {
    //AUTO DISTANCES
    public static final double WOOFERFRONT_TO_NOTE = 60;
    public static final double WOOFER_PULL_AWAY = 14;
    public static final double PULL_AWAY_TO_NOTE = 67;//71 
    public static final double WOOFER_ANGLED_TO_NOTE = 14;
    public static final double NOTE_TO_NOTE = 57;
    public static final double NOTE_TO_MIDFLD = 206; //160  //206
    //TURNING ANGLES
    public static final double TURN_ANGLE_BLUE_POD_TO_SPKR = 37;//30
    public static final double TURN_ANGLE_RED_POD_TO_SPKR = 37;//30
    public static final double TURN_ANGLE_BLUE_POD_TO_SPKR_HIGH_GEAR = 30;
    public static final double TURN_ANGLE_RED_POD_TO_SPKR_HIGH_GEAR= 30;
    public static final double TURN_SIDE_OF_WOOFER = 60;//58;//65
    //lets us ignore small joystick inputs
    public static final double LEFT_DEADZONE = 0.05; 
    public static final double RIGHT_DEADZONE = 0.05;
    public static final double DEADBAND = 0.05;
    //Transmission solenoid
    public static final int SOL_LOW_GEAR = 0; 
    public static final int SOL_HIGH_GEAR = 1;
    //External drive encoders
    public static final int DIO_LDRIVE_ENC_A = 6;
    public static final int DIO_LDRIVE_ENC_B = 7;
    public static final int DIO_RDRIVE_ENC_A = 11; //changed!
    public static final int DIO_RDRIVE_ENC_B = 10; //changed!
    //AUTO SWITCHES
    public static final int DIO_AUTO_1 = 0;
    public static final int DIO_AUTO_2 = 1;
    public static final int DIO_AUTO_3 = 2;
    public static final int DIO_AUTO_4 = 3;
    //Calculates distance in INCHES from encoder pulses (ticks get it?)
    public static final double DIAMETER = 3.8825;//3.74// last set to 3.92;  //diameter of the wheels
    public static final double CIRCUMFERENCE = Math.PI * DIAMETER;

    public static final double GEAR_RATIO = 1; //for external encoder - no gear ratio
    public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
    public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;
    public static final double DISTANCE_PER_PULSE_K = REV_TO_IN_K/512; // for external encoder
    //PID
    public static final double KP_DRIVE = 0.013; //.0153******* 
    public static final double KI_DRIVE = 0;
    public static final double KD_DRIVE = 0;

    public static final double KP_GYRO_TURN_CW = 0.02;//TUNE _ THIS IS A GUESS
    public static final double KP_GYRO_TURN_CCW = 0.02;

    public static final double KP_TURN_CCW = 0.03;//0.0223;//******
    public static final double TURNCCW_DEG_TO_INCHES = 0.221;// 0.2151;
    public static final double KP_TURN_CW = 0.03;//0.035;//0.0222; //*******%
    public static final double TURNCW_DEG_TO_INCHES = 0.221;// 0.228;//0.213; 
  }
  public static class Elevator {
    public static final int DIO_ELEV_TOP = 4;
    public static final int DIO_ELEV_BOTTOM = 5;

    public static final double JUST_ABOVE_CHAIN_HEIGHT = 22.8;
    public static final double MATCH_HEIGHT= 4;
    public static final double MIN_HEIGHT = 1.5; 
    public static final double MAX_HEIGHT = 27;
    public static final double CLIMB_HEIGHT = 1.5; 

    public static final double ELEV_REV_TO_IN = 0.32327;
    public static final double ELEV_IN_TO_REV = 1/(0.32327);
    // manual speeds
    public static final double ELEV_UP_SPEED = 0.6;
    public static final double ELEV_DOWN_SPEED = 0.4; //keep this positive
    public static final double ELEV_MAN_DOWN_SPEED = 0.8;//without PID
    // PID may need seperate pid for up, down, and climb
    public static final double KP_ELEV_UP = 0.2; //
    public static final double KI_ELEV_UP = 0;
    public static final double KD_ELEV_UP = 0;
    public static final double KFF_ELEV_UP = 0;//only use KFF with velocity control

    public static final double KP_ELEV_DOWN = 0.2;
    public static final double KI_ELEV_DOWN = 0;
    public static final double KD_ELEV_DOWN = 0;
    public static final double KFF_ELEV_DOWN = 0;

    public static final double KP_ELEV_CLIMB = 0.2;
    public static final double KI_ELEV_CLIMB = 0;
    public static final double KD_ELEV_CLIMB = 0;
    public static final double KFF_ELEV_CLIMB = 0;
    //transmission solenoid
    public static final int SOL_BRAKE_ON = 2; 
    public static final int SOL_BRAKE_OFF = 3;
  }
  public static class Intake {
    public static final int DIO_COUNTER = 12;
    public static final double INTAKE_SPEED = 0.5;//TODO experiment with this speed
    public static final double EJECT_SPEED = -0.5;
  }
  public static class CartridgeShooter { 
    //CARTRIDGE SHOOTER MOTORS
    public static final double PODIUM_PID_LEFT_RPM = 3600;//3500 wiht kFF = 0.0002, 4500 with kFF = 0.00025
    public static final double PODIUM_PID_RIGHT_RPM = 3600;
    public static final double WOOFER_PID_LEFT_RPM = 3000;//1900;//4500 with kFF = 0.00025
    public static final double WOOFER_PID_RIGHT_RPM = 3000;//1900;//4500 with kFF = 0.00025
    public static final double AMP_PID_LEFT_RPM = 1200;//cart speed for AMP shot
    public static final double AMP_PID_RIGHT_RPM = 1200;//cart speed for AMP shot
    public static final double MAX_PID_SPEED = 5500;
     //manual speeds
    public static final double WOOFER_SHOT_MOTOR_SPEED = 0.6;//for bench test - use PID in match
    public static final double PODIUM_SHOT_MOTOR_SPEED = 0.8;
    public static final double MANUAL_SET_SPEED = 0.3; 
     //pid (2022 pid constants commented in)
    public static final double kPLeft = 0.00005;  
    public static final double kILeft = 0.0; // 
    public static final double kDLeft = 0.0565; 
    public static final double kFFLeft = 0.0002;//0.0002; //0.00025 for 4500 RPM //0.0002 for 3500
    public static final double kFFWooferLeft = 0.00018;
    public static final double kPRight = 0.00005; 
    public static final double kIRight = 0.0; 
    public static final double kDRight = 0.0565; 
    public static final double kFFRight = 0.0002;//0.0002; //0.00025 for 4500 RPM //0.0002 for 3500
    public static final double kFFWooferRight = 0.00018;
  }
  public static class Tilt { 
    //TILT MOTOR
    public static final int DIO_TILT_EXT_LIMIT = 8;
    public static final int DIO_TILT_RET_LIMIT = 9;
    public static final double TILT_ENC_REVS_MAX = -74;//really-79.8, 64deg from top
    public static final double TILT_ENC_REVS_STOW = 0; //leave at zero
    public static final double TILT_ENC_REVS_WOOFER = -19;//-21; //19 //WAS POSITIVE
    public static final double TILT_ENC_REVS_PODIUM = -50;//47;  WAS POSITITVE
    public static final double TILT_ENC_REVS_SIDE_NOTE = -52;
    public static final double TILT_ENC_REVS_SIDE_NOTE_2 = -56;
    public static final double TILT_ENC_REVS_CTR_NOTE = -49;  //47; WAS POSITIVE
    public static final double MAN_EXT_SPEED = 0.2; //should be positive, the method adds correct sign
    public static final double MAN_RET_SPEED = 0.4;//should be positive the method adds correct sign
    //CARTRIDGE TILT PID:
    public static final double KP_TILT = 0.029;
    public static final double KI_TILT = 0;
    public static final double KD_TILT = 0;
    public static final double KFF_TILT = 0;
  }

  public static class Thrustmaster {
    public static final int TRIGGER = 1;
    public static final int BUTTON_MIDDLE = 2;
    public static final int BUTTON_LEFT = 3;
    public static final int BUTTON_RIGHT = 4;
    public static final int LEFT_BASE_1 = 11;
    public static final int LEFT_BASE_2 = 16;
    public static final int LEFT_BASE_3 = 13;
    public static final int LEFT_BASE_4 = 14;
    public static final int RIGHT_BASE_5 = 7;
    public static final int RIGHT_BASE_6 = 8;
    public static final int RIGHT_BASE_7 = 5;
    public static final int RIGHT_BASE_8 = 10;

    public static class AxesThrustmaster {
      public static final int X = 0;
      public static final int Y = 1;
      public static final int Z = 2;
      public static final int THROTTLE = 3;
    }
  }

  public static class XboxController {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int VIEW = 7;
    public static final int MENU = 8;
    public static final int LM = 9;
    public static final int RM = 10;

    public static class AxesXbox {
      public static final int LX = 0;
      public static final int LY = 1;
      public static final int LTrig = 2;
      public static final int RTrig = 3;
      public static final int RX = 4;
      public static final int RY = 5;
    }

    public class POVXbox {
      public static final int UP_ANGLE = 0;
      public static final int RIGHT_ANGLE = 90;
      public static final int DOWN_ANGLE = 180;
      public static final int LEFT_ANGLE = 270;
    }
  }

    public static class LogitechF310 {
      // ****when controller is in DirectInput mode (use slider on the back of the controller)
      public static final int A = 2;
      public static final int B = 3;
      public static final int X = 1;
      public static final int Y = 4;
      public static final int LB = 5;
      public static final int RB = 6;
      public static final int BACK = 9;
      public static final int START = 10;
      public static final int LEFT_PRESS = 7;
      public static final int RIGHT_PRESS = 8;
      public class AxesController {
          public static final int LEFT_X = 0;
          public static final int LEFT_Y = 1;
          public static final int LT = 2;
          public static final int RT = 3;
          public static final int RIGHT_X = 4;
          public static final int RIGHT_Y = 5;
      }
    }

}
