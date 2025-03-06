// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public boolean motorOverride = false;

  public Encoder shaftEncoder = new Encoder(ElevatorConstants.SHAFT_ENCODER_CHANNEL_A, ElevatorConstants.SHAFT_ENCODER_CHANNEL_B);
  //private double encoderOffset = ElevatorConstants.ENCODER_OFFSET;

  public TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
  public TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);

  public PIDController elevatorPID = new PIDController(0.1, 0, 0);
  public ElevatorPosition elevatorAimPos = ElevatorPosition.L1;

  public ElevatorSubsystem() {
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

    if (ElevatorConstants.ELEVATOR_DEBUG) {
      SmartDashboard.putData("Elevator PID", elevatorPID);
    }
  }

  public double getMotorOutputPower() { // Calculate motor output power using a PID Controller
    if (motorOverride) return 0;
    double output = elevatorPID.calculate(shaftEncoder.get(), elevatorAimPos.aim);
    return MathUtil.clamp(output, -1, 1);
  }

  public enum ElevatorPosition { // Enums for elevator positions
    L0(ElevatorConstants.L0), // Aka ground
    L1(ElevatorConstants.L1),
    L2(ElevatorConstants.L2),
    L3(ElevatorConstants.L3),
    L4(ElevatorConstants.L4),
    Lmax(ElevatorConstants.L4);

    public final double aim;
    
    ElevatorPosition(double position) {
      this.aim = position;
    }
  }

  @Override
  public void periodic() {}
}