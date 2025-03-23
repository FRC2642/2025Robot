// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public boolean motorOverride = false;

  ElevatorArmSubsystem armSubsystem;
  
  public Encoder shaftEncoder = new Encoder(ElevatorConstants.SHAFT_ENCODER_CHANNEL_A, ElevatorConstants.SHAFT_ENCODER_CHANNEL_B);
  public DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);

  public TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID);
  public TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID);

  public PIDController elevatorPID = new PIDController(0.1, 0, 0);
  public ElevatorPosition elevatorAimPos = ElevatorPosition.L0;

  public XboxController control;

  public ElevatorSubsystem(XboxController controller, ElevatorArmSubsystem subsystem) {
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    this.control = controller;
    this.armSubsystem = subsystem;

    if (ElevatorConstants.ELEVATOR_DEBUG) {
      SmartDashboard.putData("Elevator PID", elevatorPID);
      SmartDashboard.putBoolean("Arm Override", interruptArmRelative(false));
    }
  }

  public double getEncoderValue() { return shaftEncoder.get()/1000.0; }

  public double getArmEncoderValue() { return armSubsystem.shaftEncoder.get(); }
  public boolean interruptArmRelative(boolean rumble) {
    boolean bool = getArmEncoderValue() < .35;
    if (rumble) {
      if (bool) control.setRumble(RumbleType.kRightRumble, 0.05);
      else control.setRumble(RumbleType.kRightRumble, 0);
    }
    return bool;
  }

  /** Calculates and returns the motor output power. DO NOT USE MORE THAN ONCE IN A FRAME. */
  public double getMotorOutputPower() {
    evalLimitSwitch();
    if (interruptArmRelative(true)) return 0;
    else if (motorOverride) {
      if (control.getRightBumperButtonPressed()) {motorOverride = false; shaftEncoder.reset();}
      if (control.getLeftTriggerAxis() > 0.1) return -control.getLeftTriggerAxis()/5;
      else return 0;
    } else {
      if (control.getRightBumperButtonPressed()) { motorOverride = true; return 0; }
      double output = elevatorPID.calculate(getEncoderValue(), elevatorAimPos.aim);
      return MathUtil.clamp(output, -1, 1);
    }
  }

  public void evalLimitSwitch() {
    if (!limitSwitch.get()) shaftEncoder.reset();
  }

  /**
   * Manual power input to both motors. Positive = up & negative = down.
   * @param power
   */
  public void setMotors(double power) {
    rightElevatorMotor.set(power);
    leftElevatorMotor.set(-power);
  }

  /** Automatically sets motors based on subsystem methods.*/
  public void autoSetMotors() {
    setMotors(getMotorOutputPower());
  }

  public enum ElevatorPosition { // Enums for elevator positions
    L0(ElevatorConstants.L0), // Aka ground
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
    //System.out.println(shaftEncoder.get());
  }
}