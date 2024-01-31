// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Controller {
    // usb port on the laptop for the Xbox controller used by the Controller
    // Operator (not driver)
    // public static final int USB_AUXCONTROLLER = 1;

    // usb port on the laptop when driver using XBox controller
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;

    // for using Joysticks vice XBox for driving
    // usb ports on the laptop, when driver using two Thrustmaster Joysticks
    // public static final int USB_LEFT_JOYSTICK = 1;
    // public static final int USB_RIGHT_JOYSTICK = 2;
  }

  // these are the id numbers for the motor controllers
  public static class MotorControllers {

    public static final int ID_LEFT_FRONT = 42; // 35
    public static final int ID_RIGHT_FRONT = 43; // 1
    public static final int ID_LEFT_REAR = 41; // 34
    public static final int ID_RIGHT_REAR = 46; // 32

    public static final int ID_INTAKE_LEFT = 35; // TODO find IDs for left/right controllers
    public static final int ID_INTAKE_RIGHT = 34;

    public static final int ID_AMP_TRAP_SHOOTER = 33; // TODO get real number

    // Elevator
    public static final int ID_ELEVATOR_LEFT = 1;// TODO get real number
    public static final int ID_ELEVATOR_RIGHT = 9;// TODO get real number

    // Cartridge shooter
    public static final int ID_SHOOTER_LEFT = 10; // 1;
    public static final int ID_SHOOTER_RIGHT = 11; // 9;

    // smart current limit
    public static final int SMART_CURRENT_LIMIT = 40;

    // Ramp rates (time in seconds to go from zero to full throttle)
    public static final double CLOSED_RAMP_RATE = 0.08;
    public static final double OPEN_RAMP_RATE = 0.08;
  }

  public static class Amp {
    public static final double AMP_TRAP_MOTOR_SPEED = 0.5; // TODO define real spead
    public static final double AMP_TRAP_MOTOR_REVERSE_SPEED = -0.5; // TODO define real spead
  }

  public static class DriveConstants {
    // lets us ignore small joystick inputs
    public static final double LEFT_DEADZONE = 0.05; // 0.15???
    public static final double RIGHT_DEADZONE = 0.05;
    public static final double DEADBAND = 0.05;

    // Transmission solenoid
    public static final int SOL_LOW_GEAR = 4; // ***3 previously
    public static final int SOL_HIGH_GEAR = 5;// ***2 previously

    // external drive encoders
    public static final int DIO_LDRIVE_ENC_A = 18;
    public static final int DIO_LDRIVE_ENC_B = 19;
    public static final int DIO_RDRIVE_ENC_A = 9; // 13 TODO switch back after testing the Roborio DIO vs Gyro DIO
    public static final int DIO_RDRIVE_ENC_B = 8; // 12
    // Calculates distance in INCHES from encoder pulses (ticks get it?)

    public static final double DIAMETER = 6;
    public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
    public static final double GEAR_RATIO = 1; // for external encoder so no gear ratio
    public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
    public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;
    public static final double DISTANCE_PER_PULSE_K = REV_TO_IN_K / 512; // for external encoder

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
    public static final double WOOFERFRONT_TO_NOTE = 121;

    public static final double TURN_ANGLE_1 = 90;
    public static final double TURN_ANGLE_2 = -90;
  }

  public static class Elevator {
    public static final double MIDDLE_HEIGHT = 5;// TODO get real number
    public static final double BOTTOM_HEIGHT = 0.2;// TODO get real number
    public static final double TOP_HEIGHT = 30;// TODO get real number
    public static final double MAX_HEIGHT = 30.5;// prior to limit switch

    public static final int DIO_ELEV_TOP = 4;
    public static final int DIO_ELEV_BOTTOM = 5;// TODO change numbers

    public static final double ELEV_REV_TO_IN = 1; // TODO get this actual value

    public static final double ELEV_CLOSED_RAMP_RATE = 0.08;
    public static final double ELEV_OPEN_RAMP_RATE = 0.08;
    // manual speeds
    public static final double ELEV_UP_SPEED = 0.06;
    public static final double ELEV_DOWN_SPEED = 0.06; // keep this pos
    // PID may need seperate pid for up, down, and climb
    public static final double KP_ELEV_UP = 0.01; // 0.01?
    public static final double KI_ELEV_UP = 0;
    public static final double KD_ELEV_UP = 0;

    public static final double KP_ELEV_DOWN = 0.01;
    public static final double KI_ELEV_DOWN = 0;
    public static final double KD_ELEV_DOWN = 0;

    public static final double KP_ELEV_CLIMB = 0.01;
    public static final double KI_ELEV_CLIMB = 0;
    public static final double KD_ELEV_CLIMB = 0;
  }

  public static class Intake {
    // DIO

    // public static final int DIO_INTAKE_COUNTER = 10; //TODO find DIO channel for
    // counter

    public static final int DIO_COUNTER = 10; // TODO find DIO channel for counter

    // Motor
    public static final double INTAKE_SPEED = 0.2; // TODO experiment with this speed
    public static final double EJECT_SPEED = -0.2;
  }

  public static class CartridgeShooter { // *** renamed all of these cartridge constants
    // updated SOL channels to match FWD/REV and pair correctly with 2023 Robot
    // wiring
    public static final int SOL_CARTRIDGE_1_FWD = 3; // *** SHIFT HI FWD SOL from 2023Robot
    public static final int SOL_CARTRIDGE_1_REV = 2; // *** SHIFT LO REV SOL from 2023Robot
    public static final int SOL_CARTRIDGE_2_FWD = 1; // ** GRIPPER FWD SOL from 2023Robot
    public static final int SOL_CARTRIDGE_2_REV = 0; // ** GRIPPER REV SOL from 2023Robot

    public static final double PODIUM_PID_SPEED = 3000; // TODO determine speed
    public static final double WOOFER_PID_SPEED = 2100; // TODO determine speed

    public static final double WOOFER_SHOT_MOTOR_SPEED = 0.2; // ***//TODO tune the speed - use PID Velocity control
    public static final double PODIUM_SHOT_MOTOR_SPEED = 0.8;
    public static final double MANUAL_SET_SPEED = 0.1; // TODO determine speed

    public static final double MAX_PID_SPEED = 6000;

    // 2022 pid constants commented in
    public static final double kPLeft = 0.00005; // 0002
    public static final double kILeft = 0.0; // 0.00000001
    public static final double kDLeft = 0.0565; // 0.0565
    public static final double kFFLeft = 0.00018; // 0.00021

    public static final double kPRight = 0.00005; // 0002
    public static final double kIRight = 0.0; // 0.00000001
    public static final double kDRight = 0.0565; // 0.0565
    public static final double kFFRight = 0.00018; // 0.00021

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
