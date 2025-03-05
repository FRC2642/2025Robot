// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation.ShootSpeed;

public class JojoArmSubsystem extends SubsystemBase {
  private PIDController rotatePID = new PIDController(0.1, 0, 0);
  private DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(2);
  public TalonFX rotateJojoMotor = new TalonFX(21);
  public double maxRotateSpeed = 0.5;
  public TalonFX intakeJojoMotor = new TalonFX(20);
  public double maxintakeSpeed = 0.5;
  public double stopBool = 0;

  public JojoRotation jojoRotation = JojoRotation.Default;
  public JojoIntake intakeMode = JojoIntake.stop;


  public JojoArmSubsystem() {}

  public double getEncoderValue() {
    double encoderValue = shaftEncoder.get();
    
    System.out.println(encoderValue);
    return encoderValue;
  }

  public double getrotateOutput() {
    double toRotate = rotatePID.calculate(getEncoderValue(), jojoRotation.rot);
    if (toRotate > maxRotateSpeed){
      toRotate = maxRotateSpeed;
    }
    else if (toRotate < -maxRotateSpeed){
      toRotate = maxRotateSpeed;
    }
    return toRotate;
  }

  public enum JojoRotation{
    Default(-1),
    Intake(1);
    
    public final double rot;
    
    JojoRotation(double rotation) {
      this.rot = rotation;
    }
  }
  public enum JojoIntake{
    intake(1),
    stop(0);

    public final double intakeSpeed;
    JojoIntake(double speed){
      this.intakeSpeed = speed;
    }
  }

  @Override
  public void periodic() {
    intakeJojoMotor.set(intakeMode.intakeSpeed * maxintakeSpeed);
    rotateJojoMotor.set(jojoRotation.rot * stopBool * maxRotateSpeed);
  }
}
