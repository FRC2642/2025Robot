// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MathExt;
import frc.robot.Constants.ElevatorArmConstants;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmPosition.ShooterSpeed;

public class ElevatorArmSubsystem extends SubsystemBase {
  
  private DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(ElevatorArmConstants.ROTATION_MOTOR_ID);
  public double encoderValue;
  public double prevEncoderValue;
  public double encoderOffset;
  public double encoderMax = ElevatorArmConstants.ENCODER_MAX;

  public TalonFX shootMotor = new TalonFX(ElevatorArmConstants.SHOOT_MOTOR_ID);
  public TalonFX rotateMotor = new TalonFX(ElevatorArmConstants.ROTATION_MOTOR_ID);
  public boolean rotateMotorOverride = false;

  public ArmPosition armPos = ArmPosition.Default;
  public PIDController rotatePID = new PIDController(0.1, 0, 0);
  public ShooterSpeed shootSpeed = ShooterSpeed.stop;
  public PIDController shootSpdPID = new PIDController(1, 0, 0);
  public double shootSetSpeed = 0;

  public ElevatorArmSubsystem() {
    encoderValue = shaftEncoder.get();
    prevEncoderValue = encoderValue;
  }
  
  public double getEncoderValue() {
    return encoderValue;
  }

  public double getrotateOutput() {
    if (rotateMotorOverride) return 0;
    double output = rotatePID.calculate(getEncoderValue(), armPos.pos);
    return MathExt.cutValue(output, -1, 1);
  }

  public double getShooterSpeed() {
    shootSetSpeed = shootSpdPID.calculate(shootSetSpeed, shootSpeed.speed);
    return shootSetSpeed;
  }

  public enum ArmPosition{ // TO BE ADJUSTED
    Rotation1(ElevatorArmConstants.rotation1),
    Rotation2(ElevatorArmConstants.rotation2),
    Default(ElevatorArmConstants.rotation0);
    
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

  private void updateEncoderPos() {
    
  }
}

  @Override
  public void periodic() {
    shootMotor.set(getShooterSpeed());
    rotateMotor.set(getrotateOutput());
  }
}