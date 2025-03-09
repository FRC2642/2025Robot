// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX rightElevatorMotor = new TalonFX(24);
  public TalonFX leftElevatorMotor = new TalonFX(25);
  public Encoder shaftEncoder = new Encoder(0, 1);
  public DigitalInput limitSwitch = new DigitalInput(5);

  public PIDController elevatorPID = new PIDController(0.001, 0, 0);
  public double maxElevatorSpeed = 0.2;
  public ElevatorPosition elevatorPosition = ElevatorPosition.L0;

  public Trigger elevatorPositionReached = new Trigger(()-> Math.abs(getEncoderValue() - elevatorPosition.aim) < 5);
  public Trigger elevatorNearBottom = new Trigger(()-> getEncoderValue() < 1000);
  public Trigger limitReached = new Trigger(limitSwitch::get).negate();

  public ElevatorSubsystem() {
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(run(() -> {
      rightElevatorMotor.disable(); 
      leftElevatorMotor.disable(); 
      System.out.println("encoder " + getEncoderValue() + " || limitSwitch " + limitSwitch.get());
      System.out.println("Default Command Running");
    }));
  }
  
  public enum ElevatorPosition { // Enums for elevator positions
    L0(1), // Aka ground
    L1(4834),
    L2(6269),
    L3(9114),
    L4(10084),
    LM(11000); // Max is slightly higher than L4

    public final double aim;
    
    ElevatorPosition(double position) {
      this.aim = position;
    }
  }
  public double getEncoderValue(){
    double encoderValue = shaftEncoder.get();
    return encoderValue; 
  }

  public double getMotorOutput(){
    double output = elevatorPID.calculate(getEncoderValue(), elevatorPosition.aim);
    if (output > maxElevatorSpeed){
      output = maxElevatorSpeed;}
    else if (output < -maxElevatorSpeed){
      output = maxElevatorSpeed;}
    return output;
    
  }

  public Command elevatorCommand(ElevatorPosition position){
    return new RunCommand(()-> {
      elevatorPosition = position; 
      rightElevatorMotor.set(getMotorOutput()); 
      leftElevatorMotor.set(-getMotorOutput());
      System.out.println("encoder " + getEncoderValue() +  " || limitSwitch " + limitSwitch.get());})
      .until(elevatorPositionReached)
      .andThen(runOnce(() -> {
      rightElevatorMotor.disable();
      leftElevatorMotor.disable();}))
      .withName("Position Elevator");
  }

  public Command elevatorL0Command(){
    return new RunCommand(() ->{
      elevatorPosition = ElevatorPosition.L0;
      rightElevatorMotor.set(-getMotorOutput()); 
      leftElevatorMotor.set(getMotorOutput());
      System.out.println("encoder " + getEncoderValue() + " || limitSwitch " + limitSwitch.get());
    }).until(elevatorNearBottom).andThen(new RunCommand(() -> {
      System.out.println("slowDown");
      rightElevatorMotor.set(-0.05);
      leftElevatorMotor.set(0.05);})).until(elevatorPositionReached)
    .andThen(runOnce(() ->{
      shaftEncoder.reset();
      rightElevatorMotor.disable();
      leftElevatorMotor.disable();
    }));
  }

  public Command manualElevatorUpCommand(CommandXboxController control){
    return run(()-> {
      rightElevatorMotor.set(maxElevatorSpeed * control.getLeftTriggerAxis());
      leftElevatorMotor.set(-maxElevatorSpeed * control.getLeftTriggerAxis());})
      .withName("Manually Position Elevator Up");
  }

  public Command manualElevatorDownCommand(CommandXboxController control){
    return run(()-> {
      rightElevatorMotor.set(maxElevatorSpeed * -control.getRightTriggerAxis());
      leftElevatorMotor.set(-maxElevatorSpeed * -control.getRightTriggerAxis());})
      .withName("Manually Position Elevator Down");
  }

  public Command resetEncoder(){
    return runOnce(() -> {shaftEncoder.reset();});
  }
}
