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
  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int AUX_BUTTON_BOARD_PORT = 1;
    public static final int AUX_BUTTON_BOARD_2_PORT = 2;
    public static final int AUX_XBOX_CONTROLLER_PORT = 2;
  }
  
  public static class ElevatorConstants {
    public static final int SHAFT_ENCODER_CHANNEL_A = 0;
    public static final int SHAFT_ENCODER_CHANNEL_B = 1;
    public static final int ELEVATOR_MOTOR_ID1 = 30;
    public static final int ELEVATOR_MOTOR_ID2 = 31;
    
    public static final double LMAX = 11000;
    public static final double L4 = 10084;
    public static final double L3 = 9114;
    public static final double L2 = 6269;
    public static final double L1 = 4834;
    public static final double L0 = 0;

  }

  public static class SwerveModificationConstants {
    public static final boolean TURN_DEBUG = false;
    public static final double MOVEMENT_PERCENT_MODIFIER = 0.8;
  }
  public static class ElevatorArm {
    public static final int ROTATE_ARM_MOTOR = 23; //needs to be changed to actuall ID
    public static final int SHOOT_MOTOR = 22; //needs to be changed to actuall ID
    public static final int SHAFT_ENCODER_CHANNEL_A = 3; //needs to be changed to actuall ID
    public static final int SHAFT_ENCODER_CHANNEL_B = 3; //needs to be changed to actuall ID
  }
}