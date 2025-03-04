// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utilities.MathExt;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public boolean motorOverride = false;

  public DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(ElevatorConstants.SHAFT_ENCODER_CHANNEL);
  public double encoderValue;
  private double prevEncoderValue;
  private double encoderOffset = ElevatorConstants.ENCODER_OFFSET;
  private double encoderMax = 1.05; // Maximum value for encoders typically goes over 1

  private TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
  private TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);

  public PIDController elevatorPID = new PIDController(0.1, 0, 0);
  public ElevatorPosition elevatorAimPos = ElevatorPosition.L1;

  public ElevatorSubsystem() {
    encoderValue = shaftEncoder.get() + encoderOffset;
    prevEncoderValue = encoderValue;
  }
  
  public double getEncoderValue() { // Function for easy use and interchangeability
    return encoderValue;
  }

  public double getMotorOutputPower() { // Calculate motor output power using a PID Controller
    if (motorOverride) return 0;
    double output = elevatorPID.calculate(getEncoderValue(), elevatorAimPos.aim);
    return MathExt.cutValue(output, -1, 1);
  }

  public enum ElevatorPosition { // Enums for elevator positions
    L0(ElevatorConstants.L0),
    L1(ElevatorConstants.L1),
    L2(ElevatorConstants.L2),
    L3(ElevatorConstants.L3),
    L4(ElevatorConstants.L4);

    public final double aim;
    
    ElevatorPosition(double position) {
      this.aim = position;
    }
  }

  public void updateEncoderPos() {
    encoderValue = shaftEncoder.get() + encoderOffset; // Dynamically modify encoder values to prevent problems with the encoder reseting
    if (encoderValue - prevEncoderValue > 0.8) encoderOffset += encoderMax;
    else if (prevEncoderValue - encoderValue > 0.8) encoderOffset -= encoderMax;
    prevEncoderValue = encoderValue;
  }

  @Override
  public void periodic() {
    updateEncoderPos();

    rightElevatorMotor.set(-getMotorOutputPower());
    leftElevatorMotor.set(getMotorOutputPower());
  }
}