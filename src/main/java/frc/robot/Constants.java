// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Controller {
    //these are the usb ports on the laptop
    public static final int USB_AUXCONTROLLER = 1;
    public static final int USB_DRIVECONTROLLER = 0;
  }

    //these are the id numbers for the motor controllers
    public static class MotorControllers {
      public static final int ID_LEFT_FRONT = 35; 
      public static final int ID_RIGHT_FRONT = 1; 
      public static final int ID_LEFT_REAR = 34;
      public static final int ID_RIGHT_REAR = 32;
      }
  
  public static class DriveConstants {
    //lets us ignore small joystick inputs
    public static final double LEFT_DEADZONE = 0.05; //0.15???
    public static final double RIGHT_DEADZONE = 0.05;

    public static final double SENSITIVITY_X = 0.9;
    public static final double SENSITIVITY_Y = 0.9; //TODO  try 0.75
    //public static final boolean IS_DEADZONE = true;

    public static final int DIO_LDRIVE_ENC_A = 0;
    public static final int DIO_LDRIVE_ENC_B = 1;
    public static final int DIO_RDRIVE_ENC_A = 2;
    public static final int DIO_RDRIVE_ENC_B = 3;

    public static final double DIAMETER = 6;
    public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
    public static final double GEAR_RATIO = 1;
    public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;
    public static final double IN_TO_REV_K = GEAR_RATIO / CIRCUMFERENCE;
    public static final double DISTANCE_PER_PULSE_K = REV_TO_IN_K/512;
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

