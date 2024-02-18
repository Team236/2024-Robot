
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
public final class Constants {
  //camera servo motor, created in Robot.java:
  public static final int PWM_FRONT_CAM = 0;
  //for aiming camera at Amp/Trap or at Floor (from 0 to 1, for 0 to 180 degrees)
  public static final double FRONT_CAM_AMP = 0.5;  //TODO adjust these as needed
  public static final double FRONT_CAM_FLOOR = 0.8; //TODO adjust as needed

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

    public static final int ID_CARTRIDGE_TILT = 51; 

    public static final int SMART_CURRENT_LIMIT = 40;
    //Ramp rates (time in seconds to go from zero to full throttle)
    public static final double CLOSED_RAMP_RATE = 0.08;
    public static final double OPEN_RAMP_RATE = 0.08;
      }
  
  public static class Amp{
    public static final double AMP_TRAP_MOTOR_SPEED = 1.0; // TODO define real spead
    public static final double AMP_TRAP_MOTOR_REVERSE_SPEED =-0.4; // TODO define real spead
    }

  public static class DriveConstants {
    // lets us ignore small joystick inputs
    public static final double LEFT_DEADZONE = 0.05; // 0.15???
    public static final double RIGHT_DEADZONE = 0.05;
    public static final double DEADBAND = 0.05;
    // Transmission solenoid
    public static final int SOL_LOW_GEAR = 0; 
    public static final int SOL_HIGH_GEAR = 1;
    // external drive encoders
    public static final int DIO_LDRIVE_ENC_A = 6;
    public static final int DIO_LDRIVE_ENC_B = 7;
    public static final int DIO_RDRIVE_ENC_A = 11; //changed!
    public static final int DIO_RDRIVE_ENC_B = 10; //changed!
    // Calculates distance in INCHES from encoder pulses (ticks get it?)
    public static final double DIAMETER = 6;
    public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
    public static final double GEAR_RATIO = 1; //for external encoder - no gear ratio
    public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
    public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;
    public static final double DISTANCE_PER_PULSE_K = REV_TO_IN_K/512; // for external encoder
    // PID
    public static final double KP_DRIVE = 0.022; // 0.022 (from 2023)
    public static final double KI_DRIVE = 0;
    public static final double KD_DRIVE = 0;
    public static final double KP_TURNL = 0.025;
    public static final double PID_L_SETPOINT = 0.28;
    public static final double KP_TURNR = 0.025;
    public static final double PID_R_SETPOINT = 0.28;
    // AUTO DISTANCES
    public static final double AUTO_DISTANCE_1 = 36;
    public static final double WOOFERFRONT_TO_NOTE = 60;
    public static final double WOOFER_PULL_AWAY = 10;
    public static final double PULL_AWAY_TO_NOTE = 60;
    // TURNING ANGLES
    public static final double TURN_ANGLE_1 = 180;
    public static final double TURN_ANGLE_2 = -180;
    public static final double TURN_SIDE_OF_WOOFER = 57;
    // TIMEOUTS (in sec)
    public static final double CARTRIDGE_SHOOT_TIMEOUT = 2;
    //AUTO SWITCHES
    public static final int DIO_AUTO_1 = 0;
    public static final int DIO_AUTO_2 = 1;
    public static final int DIO_AUTO_3 = 2;
    public static final int DIO_AUTO_4 = 3;
  }
  public static class Elevator {
    public static final double JUST_ABOVE_CHAIN_HEIGHT = 20;// TODO get real number-somewhere above chain
    public static final double MATCH_HEIGHT= 8; // TODO determine, ideally same as just-above-chain
    public static final double MIN_HEIGHT = 1.0; 
    public static final double MAX_HEIGHT = 27; 
    public static final double CLIMB_HEIGHT = 1.0;

    public static final int DIO_ELEV_TOP = 4;
    public static final int DIO_ELEV_BOTTOM = 5;

    public static final double ELEV_REV_TO_IN = 0.32327;//gear ratio changed, old 2.2629;
    public static final double ELEV_IN_TO_REV = 1/(0.32327);

    public static final double ELEV_CLOSED_RAMP_RATE = 0.08;
    public static final double ELEV_OPEN_RAMP_RATE = 0.08;
    // manual speeds
    public static final double ELEV_UP_SPEED = 0.3;
    public static final double ELEV_DOWN_SPEED = 0.3; // keep this pos, use 0.5 to test climb manually
    public static final double ELEV_MAN_DOWN_SPEED = 0.8;
    // PID may need seperate pid for up, down, and climb
    public static final double KP_ELEV_UP = 0.03; //
    public static final double KI_ELEV_UP = 0;
    public static final double KD_ELEV_UP = 0;
    public static final double KFF_ELEV_UP = 0;//only use KFF with velocity control

    public static final double KP_ELEV_DOWN = 0.03;
    public static final double KI_ELEV_DOWN = 0;
    public static final double KD_ELEV_DOWN = 0;
      public static final double KFF_ELEV_DOWN = 0;

    public static final double KP_ELEV_CLIMB = 0.03;
    public static final double KI_ELEV_CLIMB = 0;
    public static final double KD_ELEV_CLIMB = 0;
      public static final double KFF_ELEV_CLIMB = 0;
  }
  public static class Intake {
    public static final int DIO_COUNTER = 12;
    public static final double INTAKE_SPEED = 0.7; // TODO experiment with this speed
    public static final double EJECT_SPEED = -0.5;
  }
  public static class CartridgeShooter { 
    // SHOOTER MOTORS
    public static final double PODIUM_PID_RPM = 1500; //TODO determine speed
    public static final double WOOFER_PID_RPM = 2100; //TODO determine speed
    public static final double AMP_PID_RPM = 2000; //TODO determine speed
    public static final double MAX_PID_SPEED = 6000;

    public static final double WOOFER_SHOT_MOTOR_SPEED = 0.2; //TODO determine these speeds
    public static final double PODIUM_SHOT_MOTOR_SPEED = 0.8;
    public static final double MANUAL_SET_SPEED = 0.3; 
   //CartridgeShooterMotor PID // 2022 pid constants commented in
    public static final double kPLeft = 0.00005; // 0002
    public static final double kILeft = 0.0; // 0.00000001
    public static final double kDLeft = 0.0565; // 0.0565
    public static final double kFFLeft = 0.00018; // 0.00021

    public static final double kPRight = 0.00005; // 0002
    public static final double kIRight = 0.0; // 0.00000001
    public static final double kDRight = 0.0565; // 0.0565
    public static final double kFFRight = 0.00018; // 0.00021
  }
  public static class Tilt { 
    //TILT MOTOR STUFF:
    public static final int DIO_TILT_EXT_LIMIT = 8;
    public static final int DIO_TILT_RET_LIMIT = 9;
    public static final double MAX_TILT_ENC_REVS = 78;  //really 79.8, 64deg from top
    public static final double TILT_ENC_REVS_WOOFER = 26.85; //34.4 deg//TODO FIND REAL ENCODER VALUES
    public static final double TILT_ENC_REVS_PODIUM = 46.71; //47 deg from top
    public static final double TILT_ENC_REVS_STOW = 0; //leave this one at zero
    public static final double MAN_EXT_SPEED = 0.1; //TODO find these EXT/RET speeds
    public static final double MAN_RET_SPEED = 0.04;  //should be positive, the method adds the negative
    //CARTRIDGE TILT PID:
    public static final double KP_TILT = 0.015;
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

}
