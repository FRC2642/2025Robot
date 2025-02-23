// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MathR;

public class JojoArmSubsystem extends SubsystemBase {
  
  private final int TILT_TOLERANCE = 0;

  private PIDController tiltPID = new PIDController(0, 0, 0);

  private TalonFX cylinderMotor = new TalonFX(21);
  private static TalonFX jojoPivot = new TalonFX(20);

  private static DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(0);

  private IntakePosition currentSetPosition = IntakePosition.RETRACTED;
  private double speedLimit = 0.2;

  public static final double MAX_DEGREES = 100;
  public static final double MIN_DEGREES = 0;

  public static final double INTAKE_TILT_ENCODER_MAX_VALUE = 1;
  public static final double INTAKE_TILT_ENCODER_MIN_VALUE = 0;
  public static final double INTAKE_TILT_ENCODER_OFFSET = 0;

  public JojoArmSubsystem() {
    //Set the Jojo arm tilt tolerance
    tiltPID.setTolerance(TILT_TOLERANCE);
  }

  public static double getPitch() {
    return MathR.getDistanceToAngle(0, tiltEncoder.getAbsolutePosition() / (INTAKE_TILT_ENCODER_MAX_VALUE - INTAKE_TILT_ENCODER_MIN_VALUE) * 360 + INTAKE_TILT_ENCODER_OFFSET, 180);
  }

  public void tiltToAngle(double degrees) {
    double power = MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1);

    //jojoPivot.setControl(power, true, false, false, false);
    jojoPivot.set(MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), degrees), 0), -1, 1));
  }

  //Set intake speed
  public void setIntake(double speed) {
    cylinderMotor.setControl(new DutyCycleOut(speed));
  }

  //Set the jojo arm movent to a certain speed
  public void set(double speed) {
    currentSetPosition = IntakePosition.MANUAL;

    jojoPivot.set(speed);
  }

  //Set the jojo arm to a certain named position
  public void set(IntakePosition pos) {
    tiltPID.setSetpoint(0);
    double speed = -MathR.limit(tiltPID.calculate(MathR.getDistanceToAngle(getPitch(), pos.angle)), -speedLimit, speedLimit);

    if (speed > 0 && getPitch() >= MAX_DEGREES) {
      speed = 0;
    }
    else if (speed < 0 && getPitch() <= MIN_DEGREES) {
      speed = 0;
    }

    if (!atSetPosition())
      set(speed);
    else
      set(0.0);
    
    currentSetPosition = pos;
  }

  //Run the jojo pivot
  public void setManual(double speed) {
    jojoPivot.set(speed);
  }

  //Check if the intake is at a certain position within a tolerance
  public boolean atSetPosition() {
    return tiltPID.atSetpoint();
  }

  //Return the intake's current set position
  public IntakePosition getSetPosition() {
    return currentSetPosition;
  }

  //Set the intake tilt speed limit
  public void setSpeedLimit(double max) {
    speedLimit = max;
  }



  //Intake angle names
  public enum IntakePosition {
    RETRACTED(100),
    EXTENDED(1),
    MANUAL(-1);

    public final double angle;
    private IntakePosition(double angle) {
      this.angle = angle;
    }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}