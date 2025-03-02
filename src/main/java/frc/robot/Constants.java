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
  }
  
  public static class ElevatorConstants {
    public static final int SHAFT_ENCODER_CHANNEL = 0;
    public static final double ENCODER_OFFSET = 0;
    public static final double ENCODER_MAX = 1.05;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 24;
    public static final int LEFT_ELEVATOR_MOTOR_ID = 25;

    // Reef encoder measurements
    public static final double L4 = 10;
    public static final double L3 = 6;
    public static final double L2 = 3;
    public static final double L1 = 1;
    public static final double L0 = 0;
  }

  public static class SwerveModificationConstants {
    public static final boolean TURN_DEBUG = false;
    public static final double PID_KP = 0.1;
    public static final double MOVEMENT_PERCENT_MODIFIER = 0.9;
  }

  public static class ElevatorArmConstants {
    public static final int SHAFT_ENCODER_CHANNEL = 1;
    public static final double ENCODER_MAX = 1.05;
    public static final int ROTATION_MOTOR_ID = 32;
    public static final int SHOOT_MOTOR_ID = 33;

    // Arm Encoder measurements
    public static final double rotation0 = 0;
    public static final double rotation1 = 0.5;
    public static final double rotation2 = 0.75;
  }
}