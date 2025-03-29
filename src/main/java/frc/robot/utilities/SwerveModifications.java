// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModificationConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Enlists rotational improvements to CTRE Swerve.
 */
public class SwerveModifications {
  public boolean turnDebug = SwerveModificationConstants.TURN_DEBUG;
  public double rotationOffset;

  public DynamicController rotationController = new DynamicController(20, 1/30, true);
  public double movementPercentModifier = SwerveModificationConstants.MOVEMENT_PERCENT_MODIFIER;

  private CommandSwerveDrivetrain drivetrain;
  XboxController control;

  public double calculatedAngleDiff;
  public double joystickDeadzone;

  public SwerveModifications(CommandSwerveDrivetrain drivetrainIN, XboxController controller, double deadzone) { // In the future, include a way to input an offset to prevent odd orientating after running an auto
    this.drivetrain = drivetrainIN;
    this.control = controller;
    this.rotationOffset = this.getRotationOffset();
    this.joystickDeadzone = deadzone;

    if (turnDebug) {
      SmartDashboard.putNumber("Swerve Rotation Calculated Angle Diff", calculatedAngleDiff);
    }
  }

  public double recieveTurnRate(Translation2d inputPos) {
    // Reset rotation offset 
    //if (control.getLeftBumperButtonPressed()) rotationOffset = getRotationOffset();

    /* Convert controller left stick x and y to degrees (0 - 360) */
    double angle = Math.atan2(inputPos.getY(), inputPos.getX());
    angle *= 180/Math.PI;
    angle += 90;
    angle = SectorLimit(angle, RotationSector.normal);

    double currentAngle = drivetrain.getPigeon2().getRotation2d().getDegrees();
    currentAngle = currentAngle % 360;
    angle = SectorLimit(angle, RotationSector.normal);
    currentAngle *= -1;
    currentAngle += rotationOffset;
    angle = SectorLimit(angle, RotationSector.normal);

    if (absDiff(angle, currentAngle) > 180) angle = SectorLimit(angle, RotationSector.forwardBased);

    if (absDiff(angle, currentAngle) > 180) System.out.println("WARNING: HIGH CALCULATED ANGLE");

    if (new Translation2d(control.getRightX(), control.getRightY()).getNorm() >= joystickDeadzone) return rotationController.calculateOutput(currentAngle, angle); // Joystick mag >= joystic deadzone
    else {
      rotationController.updateOutputs(0); // Should fix sudden movements due to 
      return 0;
    }
  }

  public double getRotationOffset() {
    double stateRotation = drivetrain.getState().Pose.getRotation().getDegrees();
    double gyroRotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    gyroRotation %= 360;
    if (gyroRotation > 180) gyroRotation -= 360;
    if (gyroRotation < -180) gyroRotation += 360;
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

  public enum RotationSector {
    normal(0, 360),
    forwardBased(-180, 180);

    public final double lowerLimit;
    public final double upperLimit;

    RotationSector(double lower, double upper) {
      this.lowerLimit = lower;
      this.upperLimit = upper;
    }
  }

  public double SectorLimit(double value, RotationSector sector) {
    if (value < sector.lowerLimit) value += 360;
    else if (value > sector.upperLimit) value -= 360;
    return value;
  }
}
