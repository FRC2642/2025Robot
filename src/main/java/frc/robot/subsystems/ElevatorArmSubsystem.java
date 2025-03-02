// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation.ShootSpeed;

public class ElevatorArmSubsystem extends SubsystemBase {
  
  private PIDController rotatePID = new PIDController(0.1, 0, 0);
  private Encoder shaftEncoder = new Encoder(Constants.ElevatorArm.SHAFT_ENCODER_CHANNEL_A, Constants.ElevatorArm.SHAFT_ENCODER_CHANNEL_B);
  public TalonFX shootMotor = new TalonFX(Constants.ElevatorArm.SHOOT_MOTOR);
  public TalonFX rotateMotor = new TalonFX(Constants.ElevatorArm.ROTATE_ARM_MOTOR);
  public double maxShootOutput= -0.25;

  public ArmRotation armRot = ArmRotation.Default;
  public ShootSpeed shootSpeed = ShootSpeed.stop;

  public ElevatorArmSubsystem() {
  }

  public void resetEncoder() {
    shaftEncoder.reset();
  }
  public double getEncoderValue() {
    return shaftEncoder.getDistance();
  }
  public double getrotateOutput() {
    return rotatePID.calculate(getEncoderValue(), armRot.rot);
  }

  public enum ArmRotation{
    Rotation1(90),
    Rotation2(120),
    Default(0);
    
    public final double rot;

    ArmRotation(double rotation) {
      this.rot = rotation;
  }
  public enum ShootSpeed{
    intake(-1),
    shoot(1),
    stop(0);

    public final double speed;

    ShootSpeed(double speed){
      this.speed = speed;
    }
  }
}

  @Override
  public void periodic() {
    shootMotor.set(shootSpeed.speed * maxShootOutput);
    //rotateMotor.set(getrotateOutput());
  }
}
