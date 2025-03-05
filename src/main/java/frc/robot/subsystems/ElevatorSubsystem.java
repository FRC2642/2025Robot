// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public Encoder shaftEncoder = new Encoder(Constants.Elevator.SHAFT_ENCODER_CHANNEL_A, Constants.Elevator.SHAFT_ENCODER_CHANNEL_B); // Defaults to 4X decoding (most accurate) + non-inverted
  public TalonFX elevatorMotor1 = new TalonFX(Constants.Elevator.ELEVATOR_MOTOR_ID1);
  public TalonFX elevatorMotor2 = new TalonFX(Constants.Elevator.ELEVATOR_MOTOR_ID2);

  public PIDController elevatorPID = new PIDController(0.1, 0, 0);
  public ElevatorPosition elevatorAimPos = ElevatorPosition.L1;

  public ElevatorSubsystem() {}

  public void resetEncoder() {
    shaftEncoder.reset();
  }
  
  public double getEncoderValue() {
    return shaftEncoder.getDistance();
  }

  public double getMotorOutputPower() {
    double output = elevatorPID.calculate(getEncoderValue(), elevatorAimPos.aim);
    return output;
    
  }

  public enum ElevatorPosition {
    L1(0),
    L2(0.3),
    L3(0.6),
    L4(1);

    public final double aim;
    
    ElevatorPosition(double position) {
      this.aim = position;
    }
  }

  @Override
  public void periodic() {
    //elevatorMotor1.set(getMotorOutputPower());
    //elevatorMotor2.set(getMotorOutputPower());
  }
}