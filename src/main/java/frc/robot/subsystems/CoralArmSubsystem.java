// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CoralArmSubsystem extends SubsystemBase {
  public TalonFX shootMotor = new TalonFX(Constants.ElevatorArm.SHOOT_MOTOR);
  public TalonFX rotateMotor = new TalonFX(Constants.ElevatorArm.ROTATE_ARM_MOTOR);  
  private DutyCycleEncoder shaftEncoder= new DutyCycleEncoder(Constants.ElevatorArm.SHAFT_ENCODER_CHANNEL_A);
  
  private PIDController rotatePID = new PIDController(0.5, 0, 0);

  public double maxShootOutput= 0.25;
  public double maxRotateSpeed = 0.9;
  public ArmRotation armRot = ArmRotation.Score;
  public ShootSpeed shootSpeed = ShootSpeed.stop;

  public Trigger RotationStateReached = new Trigger(() -> Math.abs(getEncoderValue() - armRot.rot) < 0.01);

  
  public CoralArmSubsystem() {
    rotateMotor.setNeutralMode(NeutralModeValue.Brake);
    shootMotor.setNeutralMode(NeutralModeValue.Brake);
    
    setDefaultCommand(runOnce(()-> {shootSpeed = ShootSpeed.stop; shootMotor.disable(); rotateMotor.disable();})
    .andThen(run(() -> {}))
    .withName("Idle"));
  }

  public enum ArmRotation{
    Score(0.27),
    Default(0.075),
    Bottom(0.572);
      public final double rot;
      ArmRotation(double rotation) {
        this.rot = rotation;}
  }
  public enum ShootSpeed{
    intake(-1),
    shoot(1),
    stop(0);
      public final double speed;
      ShootSpeed(double speed){
        this.speed = speed;}
  }

  public double getEncoderValue() {
    double encoderValue = shaftEncoder.get();
    return encoderValue;
  }
  public double getrotateOutput() {
    double toRotate = rotatePID.calculate(getEncoderValue(), armRot.rot);
    if (toRotate > maxRotateSpeed){
      toRotate = maxRotateSpeed;}
    else if (toRotate < -maxRotateSpeed){
      toRotate = maxRotateSpeed;}
    return toRotate;
  }

  public Command shootCommand(){
    return run(()-> {shootSpeed = ShootSpeed.shoot; shootMotor.set(shootSpeed.speed * maxShootOutput);})
    .withName("Shoot Coral");
  }
  public Command rotateCommand(ArmRotation armRotMode){
    return new RunCommand(()-> {armRot = armRotMode; rotateMotor.set(getrotateOutput());}).until(RotationStateReached)
    .andThen(runOnce(() -> rotateMotor.disable()))
    .withName("Rotate Coral Arm");
  }


  @Override
  public void periodic() {
  }
}
