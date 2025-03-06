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
  public static boolean DEBUG = false; // DO NOT SET TO FINAL

  public static class OperatorConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int AUX_BUTTON_BOARD_PORT = 1;
  }
  
  public static class ElevatorConstants {
    public static final boolean ELEVATOR_DEBUG = false;
    public static final int SHAFT_ENCODER_CHANNEL_A = 0;
    public static final int SHAFT_ENCODER_CHANNEL_B = 1;
    public static final double ENCODER_OFFSET = 0;
    public static final double ENCODER_MAX = 1.05;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 24;
    public static final int LEFT_ELEVATOR_MOTOR_ID = 25;

    // Reef encoder measurements
    public static final double L4 = 11000;
    public static final double L3 = 9114;
    public static final double L2 = 6269;
    public static final double L1 = 4834;
    public static final double L0 = 0;
  }

  public static class SwerveModificationConstants {
    public static final boolean TURN_DEBUG = DEBUG; // Initially seperated but for specific testing can change
    public static final double MOVEMENT_PERCENT_MODIFIER = 0.9;
  }

  public static class ElevatorArmConstants {
    public static final int SHAFT_ENCODER_CHANNEL = 3;
    public static final double ENCODER_MAX = 1.05;
    public static final int ROTATION_MOTOR_ID = 32;
    public static final int SHOOT_MOTOR_ID = 33;

    // Arm Encoder measurements
    public static final double rotation0 = 0;
    public static final double rotation1 = 0.5;
    public static final double rotation2 = 0.75;
  }

  public static class JojoConstants {
    public static final boolean JOJO_DEBUG = false;
    public static final int CYLLINDER_MOTOR_ID = 20;
    public static final int PIVOT_MOTOR_ID = 21;
    public static final int SHAFT_ENCODER_ID = 2;

    // Arm Encoder measurements
    public static final double ARM_UP = 0.27;
    public static final double ARM_OUT = 0.52;
  }
}