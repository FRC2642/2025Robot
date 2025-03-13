// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveModificationConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Enlists rotational improvements to CTRE Swerve.
 */
public class SwerveModifications {
  public boolean turnDebug = SwerveModificationConstants.TURN_DEBUG;
  public double rotationOffset;

  public DynamicController rotationController = new DynamicController(20, .033333333333333333, true);
  public double movementPercentModifier = SwerveModificationConstants.MOVEMENT_PERCENT_MODIFIER;

  private CommandSwerveDrivetrain drivetrain;
  XboxController control;

  public double calculatedAngleDiff;

  public SwerveModifications(CommandSwerveDrivetrain drivetrainIN, XboxController controller) { // In the future, include a way to input an offset to prevent odd orientating after running an auto
    this.drivetrain = drivetrainIN;
    this.control = controller;
    this.rotationOffset = this.getRotationOffset();

    if (turnDebug) {
      
    }
  }

  public double recieveTurnRate(double xAim, double yAim) {

    // Reset rotation offset 
    if (control.getRightBumperButtonPressed()) { rotationOffset = getRotationOffset(); }

    /* Convert controller left stick x and y to degrees (0 - 360) */
    double angle = Math.atan2(yAim, xAim);
    angle *= 180/Math.PI;
    angle += 90;
    if (angle > 180) angle -= 360;

    double currentAngle = drivetrain.getPigeon2().getRotation2d().getDegrees();
    currentAngle = currentAngle % 360;
    if (currentAngle > 180) currentAngle -= 360;
    else if (currentAngle < -180) currentAngle += 360;
    currentAngle *= -1;
    currentAngle += rotationOffset;
    if (currentAngle > 180) currentAngle -= 360;

    if (absDiff(angle, currentAngle) > 180) { if (angle < 0) angle += 360; else if (angle > 0) angle -= 360; }

    if (absDiff(angle, currentAngle) > 180) System.out.println("WARNING: HIGH CALCULATED ANGLE");

    double outputPower = rotationController.calculateOutput(currentAngle, angle);

    double joystickMag = Math.sqrt(Math.pow(xAim, 2) + Math.pow(yAim, 2)); // Joystick magnitude for deadzones on friction joysticks
    if (joystickMag >= 0.12) return outputPower; else return 0;
  } 

  public double getRotationOffset() {
    double stateRotation = drivetrain.getState().Pose.getRotation().getDegrees();
    double gyroRotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    gyroRotation %= 360;
    if (gyroRotation > 180) { gyroRotation -= 360; }
    return stateRotation - gyroRotation;
  }

  private double absDiff(double n1, double n2) { return Math.abs(n1 - n2); }

  /**
     * Modifies the axial input, taking in an additional input to modify the original. Returns the result.
     * 
     * @param input The original input
     * @param modifierInput The secondary, modifying input
     * @param modifyPercent The percent of the value of the original input to be affected by the modifierInput
     */

  public static double modifyAxialInput(double input, double modifierInput, double modifyPercent) {
    input = MathUtil.clamp(input, -1, 1);
    modifierInput = MathUtil.clamp(modifierInput, 0, 1);
    double output = input * (modifierInput * modifyPercent + (1 - modifyPercent));
    return output;
  }
}
