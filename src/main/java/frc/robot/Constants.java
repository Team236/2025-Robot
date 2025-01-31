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

  public static final class Controller {
    //USB port number of the xbox controllers
    public static final int USB_DRIVECONTROLLER = 0;
    public static final int USB_AUXCONTROLLER = 1;
  }

  public static class MotorControllers {
    public static final int SMART_CURRENT_LIMIT = 40;

    //Motor ID Numbers
    //Elevator (placeholder)
    public static final int ID_ELEVATOR_LEFT = 46;
    public static final int ID_ELEVATOR_RIGHT = 47;

    //AlgaeHold (placeholder)
    public static final int ID_ALGAE_HOLD = 49;

   //CoralHold (placeholder)
    // public static final int ID_CORAL_HOLD_MOTORSRX = 14; //IF USE TALON SRX
    public static final int ID_CORAL_HOLD_MOTOR = 1;//WILL BE BRUSHED SM MOTOR

    //AlgaePivot (placeholder)
    public static final int ID_ALGAE_PIVOT = 50;//Change to 50 when test algaehold

     //CoralPivot (placeholder)
     public static final int ID_CORAL_PIVOT = 2;//Change to 50 when test coralhold
  }

  public static class Elevator {
    public static final int DIO_ELEV_TOP = 4;
    public static final int DIO_ELEV_BOTTOM = 5;

    public static final double ELEV_UP_SPEED = 0.15;
    public static final double ELEV_DOWN_SPEED = -0.15;

    //placeholder conversion factors
    public static final double ELEV_REV_TO_IN = 0.32327;
    public static final double ELEV_IN_TO_REV = 1/(0.32327);

    public static final double L1_HEIGHT = 6;
    public static final double L2_HEIGHT = 12;
    public static final double L3_HEIGHT = 18;
    
    public static final double MIN_HEIGHT = 1.5;
    public static final double MAX_HEIGHT = 27;

    //placeholder PID values
    public static final double KP_ELEV = 0.2;
    public static final double KI_ELEV = 0;
    public static final double KD_ELEV = 0;
  }

public static class AlgaeHold {
  public static final double HOLD_SPEED = 0.1;
  public static final double RELEASE_SPEED = -0.1;
}

public static class CoralHold {
  public static final int DIO_COUNTER = 3;
  public static final double HOLD_SPEED = .1;
  public static final double RELEASE_SPEED = -.1;
  public static final double L4_RELEASE_SPEED = .1;
}

  public static class AlgaePivot {
    public static final int DIO_EXT_LIMIT = 8;//change to 8 when test AlgaePivot
    public static final int DIO_RET_LIMIT = 9;//change to 9 when test AlgaePivot
    public static final double ENC_REVS_MAX = -74;
    public static final double ENC_REVS_TEST1 = -30;
    public static final double ENC_REVS_TEST2 = -52;
    public static final double MAN_EXT_SPEED = -0.1;
    public static final double MAN_RET_SPEED = 0.1;
    public static final double KP = 0.029;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
  }
  
  public static class CoralPivot {
    public static final int DIO_EXT_LIMIT = 1; //change to 8 when test AlgaePivot
    public static final int DIO_RET_LIMIT = 2; //change to 9 when test AlgaePivot
    public static final double ENC_REVS_MAX = -74;
    public static final double ENC_REVS_TEST1 = -30;
    public static final double ENC_REVS_TEST2 = -52;
    public static final double MAN_EXT_SPEED = -0.1;
    public static final double MAN_RET_SPEED = 0.1;
    public static final double KP = 0.029;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KFF = 0;
    public static final int DIO_ENC_A = 11;
    public static final int DIO_ENC_B = 12;
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