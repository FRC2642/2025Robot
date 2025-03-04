// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModificationConstants;
import frc.robot.utilities.DynamicController;

public class SwerveModifications extends SubsystemBase {
  /** Creates a new SwerveModifications. */
  public boolean turnDebug = SwerveModificationConstants.TURN_DEBUG;
  private int i = 0;
  public double rotationOffset;

  public PIDController rotationPID = new PIDController(SwerveModificationConstants.PID_KP, 0, 0);
  public DynamicController dynamicController = new DynamicController(30, 0.1, true);
  public double movementPercentModifier = SwerveModificationConstants.MOVEMENT_PERCENT_MODIFIER;

  private CommandSwerveDrivetrain drivetrain;
  XboxController control;

  public SwerveModifications(CommandSwerveDrivetrain drivetrainIN, XboxController controller) { // In the future, include a way to input an offset to prevent odd orientating after running an auto
    this.drivetrain = drivetrainIN;
    this.control = controller;
    this.rotationOffset = this.getRotationOffset();
  }

  public double recieveTurnRate() {
    if (turnDebug) i++; // Iterator to prevent spam-logging

    // Reset rotation offset 
    if (control.getLeftBumperButtonPressed()) { rotationOffset = getRotationOffset(); }

    /* Convert controller left stick x and y to degrees (0 - 360) */
    double angle = Math.atan2(control.getRightY(), control.getRightX()); // Get raw positions
    angle *= 180/Math.PI; // Degrees
    angle += 90; // Turn 90 degrees to make 0 degrees up (unit circle by default)
    if (angle > 180) { angle -= 360; }
    if (i >= 10) {
      System.out.print("  " + angle); // Prints the angle to the console for debugging
    }

    double currentAngle = drivetrain.getPigeon2().getRotation2d().getDegrees();
    currentAngle = currentAngle % 360; // Reports an insanely large value by default, so % 360 to get the actual value
    if (currentAngle > 180) { currentAngle -= 360; } // Move values more than 180 to make the degrees scale from -180 to 180
    currentAngle *= -1; // It's flipped.
    currentAngle += rotationOffset; // Offset rotation
    if (currentAngle > 180) { currentAngle -= 360; }
    
    if (i >= 10) {
        System.out.println(" " + currentAngle);
        double stateAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        System.out.println(" " + stateAngle);
    }

    if (Math.abs(angle - currentAngle) > 180) { // Optimizations
        if (angle < 0) angle += 360;
        else if (angle > 0) angle -= 360;
    }

    if (Math.abs(angle - currentAngle) > 180) System.out.println("WARNING: HIGH CALCULATED ANGLE");

    double outputPower = dynamicController.calculateOutput(currentAngle, angle);

    i = (i >= 10) ? 0 : i; // Reset iterator

    double joystickMag = Math.sqrt(Math.pow(control.getRightX(), 2) + Math.pow(control.getRightY(), 2)); // Joystick magnitude for deadzones on friction joysticks
    if (joystickMag >= 0.12) {
        return outputPower;
    } else {
        return 0;
    }
  }

  public double getRotationOffset() {
    double stateRotation = drivetrain.getState().Pose.getRotation().getDegrees();
    double gyroRotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    gyroRotation %= 360;
    if (gyroRotation > 180) { gyroRotation -= 360; }
    //gyroRotation -= 90;
    //if (gyroRotation < -180) { gyroRotation += 360; }
    return stateRotation - gyroRotation;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
