// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModifications extends SubsystemBase {
  /** Creates a new SwerveModifications. */
  public boolean turnDebug = Constants.SwerveModifications.TURN_DEBUG;
  private int i = 0;
  public double rotationOffset;

  public ArrayList<Double> prevRotationOutputs = new ArrayList<>();
  public int rotationOutputListLimit = Constants.SwerveModifications.ROTATION_OUTPUT_LIST_LIMIT;
  public PIDController rotationPID = new PIDController(1.0, 0, 0);
  public double movementPercentModifier = Constants.SwerveModifications.MOVEMENT_PERCENT_MODIFIER;

  private CommandSwerveDrivetrain drivetrain;
  XboxController control;

  public SwerveModifications(CommandSwerveDrivetrain drivetrainIN, XboxController controller) {
    this.drivetrain = drivetrainIN;
    this.control = controller;
    this.rotationOffset = this.getRotationOffset();
  }

  public double recieveTurnRate() {
    if (turnDebug) { i++; } // Iterator to prevent spam-logging

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
        if (angle < 0) {
            angle += 360;
        } else if (angle > 0) {
            angle -= 360;
        }
    }

    if (Math.abs(angle - currentAngle) > 180) { System.out.println("WARNING: HIGH CALCULATED ANGLE"); }

    /*double outputPower = (angle - currentAngle) / 60; // Modifies rotational speed; retest to find the best (60)
    if (Math.abs(outputPower) > 1) { outputPower /= Math.abs(outputPower); } // Limit the output power to (abs) 1

    prevRotationOutputs.add(outputPower); // Add new values to the bottom of the list
    while (prevRotationOutputs.size() > rotationOutputListLimit) { prevRotationOutputs.remove(0); } // Remove extra data points from the top of the list
    if (prevRotationOutputs.size() == rotationOutputListLimit && outputPower >= 0.9) { // Use if we have data and the outputPower is set to max or near max
        double prevOutputAvg = 0; // Get the average power over the last 30 code runs
        for (int j = 0; j < prevRotationOutputs.size(); j++) { prevOutputAvg += Math.abs(prevRotationOutputs.get(j)); }
        prevOutputAvg /= prevRotationOutputs.size();
        outputPower *= prevOutputAvg; // The average is essentially a percent since power ranges from (abs) 0 to 1 so multiplying reduces power and creates acceleration
    }*/ // Bunch of stuff that is easily replaced by a PID:
    double outputPower = rotationPID.calculate(currentAngle, angle);

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

  /**
   * Modifies the axial input, taking in an additional input to modify the original. Returns the result.
   * 
   * @param input The original input
   * @param modifierInput The secondary, modifying input
   * @param modifyPercent The percent of the value of the original input to be affected by the modifierInput
   */

  public double modifyAxialInput(double input, double modifierInput, double modifyPercent) {
    input = cutValue(input, -1, 1);
    modifierInput = cutValue(modifierInput, 0, 1);
    double output = input * (modifierInput * modifyPercent + (1 - modifyPercent));
    // If the input is negative, made the modifier negative, and same for positive
    return output;
  }

  /**
   * Limits a value between the min and the max and returns the limited value.
   */

  public double cutValue(double value, double min, double max) {
      if (value > max) { value = max; }
      else if (value < min) { value = min; }
      return value;
  }

  /**
   * @return -1 if negative and 1 if positive
   */

  public double getSign(double value) {
      return value / Math.abs(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
