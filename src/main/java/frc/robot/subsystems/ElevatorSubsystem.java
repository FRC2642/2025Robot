// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.MathExt;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(ElevatorConstants.SHAFT_ENCODER_CHANNEL);
  public double encoderValue;
  public double prevEncoderValue;
  public double encoderOffset;
  public double encoderMax = 1.05; // Maximum value for encoders typically goes over 1

  public TalonFX elevatorMotor1 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID1);
  public TalonFX elevatorMotor2 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID2);

  public PIDController elevatorPID = new PIDController(0.1, 0, 0);
  public ElevatorPosition elevatorAimPos = ElevatorPosition.L1;

  public ElevatorSubsystem() {
    encoderValue = shaftEncoder.get();
    prevEncoderValue = encoderValue;
  }
  
  public double getEncoderValue() { // Function for easy use and interchangeability
    return encoderValue;
  }

  public double getMotorOutputPower() { // Calculate motor output power using a PID Controller
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

  @Override
  public void periodic() {
    encoderValue = shaftEncoder.get() + encoderOffset; // Dynamically modify encoder values to prevent problems with the encoder reseting
    if (encoderValue - prevEncoderValue > 0.8) encoderOffset += encoderMax;
    else if (prevEncoderValue - encoderValue > 0.8) encoderOffset -= encoderMax;
    prevEncoderValue = encoderValue;

    elevatorMotor1.set(getMotorOutputPower());
    elevatorMotor2.set(getMotorOutputPower());
  }
}