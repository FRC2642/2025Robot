// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorArmConstants;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmPosition.ShooterSpeed;

public class ElevatorArmSubsystem extends SubsystemBase {
  
  public DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(ElevatorArmConstants.SHAFT_ENCODER_CHANNEL);
  public double encoderOffset;

  public TalonFX shootMotor = new TalonFX(ElevatorArmConstants.SHOOT_MOTOR_ID);
  public TalonFX rotateMotor = new TalonFX(ElevatorArmConstants.ROTATION_MOTOR_ID);
  public boolean rotateMotorOverride = false;

  public ArmPosition armPos = ArmPosition.Retracted;
  public PIDController rotatePID = new PIDController(0.1, 0, 0);
  public ShooterSpeed shootSpeed = ShooterSpeed.stop;
  public double shootSetSpeed = 0;

  public ElevatorArmSubsystem() {
    shootMotor.setNeutralMode(NeutralModeValue.Brake);
    rotateMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getrotateOutput() {
    if (rotateMotorOverride) return 0;
    return MathUtil.clamp(rotatePID.calculate(shaftEncoder.get(), armPos.pos), -1, 1);
  }

  public enum ArmPosition{ // TO BE ADJUSTED
    Retracted(ElevatorArmConstants.RETRACTED),
    Score(ElevatorArmConstants.SCORE),
    Extended(ElevatorArmConstants.FULL_EXTEND);
    
    public final double pos;

    ArmPosition(double rotation) {
      this.pos = rotation;
  }

  public enum ShooterSpeed{
    intake(-1),
    shoot(1),
    stop(0);

    public final double speed;

    ShooterSpeed(double aimSpeed){
      this.speed = aimSpeed;
    }
  }
}

  @Override
  public void periodic() {}
}