// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JojoConstants;

public class JojoSubsystem extends SubsystemBase {
  /** Creates a new JojoSubsystem. */
  public DutyCycleEncoder encoder = new DutyCycleEncoder(JojoConstants.SHAFT_ENCODER_ID);

  public TalonFX cylinderMotor = new TalonFX(JojoConstants.CYLLINDER_MOTOR_ID);
  public TalonFX pivotMotor = new TalonFX(JojoConstants.PIVOT_MOTOR_ID);

  public PIDController pivotPID = new PIDController(0.9, 0, 0);
  public IntakePosition intakePos;
  public IntakeSpeed intakeSpeed;

  public JojoSubsystem() {
    cylinderMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    if (JojoConstants.JOJO_DEBUG) {
      SmartDashboard.putData("IntakePID", pivotPID);
    }
  }

  public double getMotorOutputPower() { // Calculate motor output power using a PID Controller
    double output = pivotPID.calculate(encoder.get(), intakePos.pos);
    return MathUtil.clamp(output, -1, 1);
  }

  public enum IntakePosition {
    Up(JojoConstants.ARM_UP),
    Out(JojoConstants.ARM_OUT);
    
    public final double pos;

    IntakePosition(double position) {
      this.pos = position;
    }
  }

  public enum IntakeSpeed {
    On(1),
    Off(0),
    Reverse(-1);

    public final double speed;

    IntakeSpeed(double speed) { this.speed = speed; }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
