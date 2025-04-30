// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX rightJojoMotor = new TalonFX(21);
  public TalonFX leftJojoMotor = new TalonFX(20);
  private DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(2);
  private double climbSpeed = 0.25;

  public ClimberSubsystem() {
    rightJojoMotor.setNeutralMode(NeutralModeValue.Brake);
    leftJojoMotor.setNeutralMode(NeutralModeValue.Brake);
    
    setDefaultCommand(run(()->{
      rightJojoMotor.set(0);
      leftJojoMotor.set(0);
    }));
  }
  public Command retract(){
    return run(()->{
      rightJojoMotor.set(-climbSpeed);
      leftJojoMotor.set(climbSpeed);
    });
  }
  public Command extend(){
    return run(()->{
      rightJojoMotor.set(climbSpeed);
      leftJojoMotor.set(-climbSpeed);
    });
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
