// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.SwerveModificationConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Enlists rotational improvements to CTRE Swerve.
 * 
 * Sorry about all the overloads... I've tried to comment as many as I could...
 */
public class SwerveModifications implements Subsystem {
  public boolean turnDebug = SwerveModificationConstants.TURN_DEBUG;
  public double rotationOffset;

  public DynamicController rotationController;
  public double movementPercentModifier = SwerveModificationConstants.MOVEMENT_PERCENT_MODIFIER;

  private CommandSwerveDrivetrain drivetrain;
  private XboxController control;
  public Translation2d movementOutput;

  public double calculatedAngleDiff;
  public double joystickDeadzone;

  public SwerveModifications(CommandSwerveDrivetrain drivetrainIN, XboxController controller, double deadzone) {
    this.rotationController = new DynamicController(20, 1/30, true);
    this.drivetrain = drivetrainIN;
    this.control = controller;
    this.resetRotationOffset();
    this.joystickDeadzone = deadzone;

    if (turnDebug) {
      SmartDashboard.putNumber("Swerve Rotation Calculated Angle Diff", calculatedAngleDiff);
    }
  }

  /**
   * Calculates the turn speed given an input direction
   * @param inputPos vector establishing direction
   * @return
   */
  public double recieveTurnRate(Translation2d inputPos) { return recieveTurnRate(inputPos, 0); } // Overload w/o offset
  /**
   * Calculates the turn speed given an input direction and rotational offset
   * @param inputPos vector establishing direction
   * @param rotationalOffset clockwise rotational offset in degrees
   * @return
   */
  public double recieveTurnRate(Translation2d inputPos, double rotationalOffset) {
    // Reset rotation offset 
    //if (control.getLeftBumperButtonPressed()) rotationOffset = getRotationOffset();

    /* Convert controller left stick x and y to degrees (0 - 360) */
    double angle = Math.atan2(inputPos.getY(), inputPos.getX());
    angle *= 180/Math.PI;
    angle += 90 + rotationalOffset;
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
      rotationController.updateOutputs(0); // Should fix sudden movements due to non-reporting
      return 0;
    }
  }
  public double recieveTurnRate(double rotationInput) {
    return rotationController.calculateOutput(rotationInput);
  }

  public double getRotationOffset() {
    double stateRotation = drivetrain.getState().Pose.getRotation().getDegrees();
    double gyroRotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
    gyroRotation %= 360;
    if (gyroRotation > 180) gyroRotation -= 360;
    if (gyroRotation < -180) gyroRotation += 360;
    return stateRotation - gyroRotation;
  }

  public void resetRotationOffset() { rotationOffset = getRotationOffset(); }

  private double absDiff(double n1, double n2) { return Math.abs(n1 - n2); }

  /**
     * Modifies the axial input, taking in an additional input to modify the original. Returns the result.
     * 
     * @param input The input to be modified
     * @param modifierInput The modifier (0-100% percent in decimal form)
     * @param modifyPercent The percent of the input to be affected by the modifierInput
     */

  public static double modifyAxialInput(double input, double modifierInput, double modifyPercent) {
    input = MathUtil.clamp(input, -1, 1);
    modifierInput = MathUtil.clamp(modifierInput, 0, 1);
    double output = input * (modifierInput * modifyPercent + (1 - modifyPercent));
    return output;
  }

  /**
   * Modifies a 2D input, taking in modifierInput and modify Percent to modify the 2D input. Returns the result.
   * @param input The 2D input to be modified
   * @param modifierInput The modifier (0-100% percent in decimal form)
   * @param modifyPercent The percent of the input to be affected by the modifierInput
   */
  public static Translation2d modifyTranslationalInput(Translation2d input, double modifierInput, double modifyPercent) {
    return new Translation2d(modifyAxialInput(input.getX(), modifierInput, modifyPercent), modifyAxialInput(input.getY(), modifierInput, modifyPercent));
  }

  /**
   * Calculates the drivetrain output given a field-relative 2D input, modifierInput, and modifyPercent. Returns the result.
   * @param input The 2D input
   * @param modifierInput The modifier (0-100% percent in decimal form)
   * @param modifyPercent The percent of the input to be affected by the modifierInput
   */
  public static Translation2d CalculateDriveOutput(Translation2d input, double modifierInput, double modifyPercent) {
    return modifyTranslationalInput(input, modifierInput, modifyPercent);
  }
  
  public static Translation2d DriveWithRotationalOffset(Translation2d input, double angularOffset) {
    Translation2d output = input;
    output.rotateBy(new Rotation2d(-angularOffset * 180 / Math.PI));
    return output;
  }
  public static Translation2d DriveWithRotationalOffset(Translation2d input, double angularOffset, double modifyPercent) {
    input = modifyTranslationalInput(input, 0, modifyPercent);
    return DriveWithRotationalOffset(input, angularOffset);
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

  /**
   * Caculates the equivalent angle within the appropriate sector.
   * @param value
   * @param sector
   * @return
   */
  public double SectorLimit(double value, RotationSector sector) {
    value %= 360;
    if (value < sector.lowerLimit) value += 360;
    else if (value > sector.upperLimit) value -= 360;
    return value;
  }

  @Override
  public void periodic() {
    movementOutput = DriveWithRotationalOffset(new Translation2d(control.getLeftX(), control.getLeftY()), 0);
  }
}
