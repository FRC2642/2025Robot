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

public class ElevatorSubsystem extends SubsystemBase {
  public TalonFX rightElevatorMotor = new TalonFX(24);
  public TalonFX leftElevatorMotor = new TalonFX(25);
  public Encoder shaftEncoder = new Encoder(0, 1);
  public DigitalInput limitSwitch = new DigitalInput(5);

  public PIDController elevatorPID = new PIDController(0.006, 0.001, 0);
  public double maxElevatorSpeed = 0.5;
  public static ElevatorPosition elevatorPosition = ElevatorPosition.L0;

  public boolean manualMode = true;

  public Trigger elevatorPositionReached = new Trigger(()-> Math.abs(getEncoderValue() - elevatorPosition.aim) < 25);
  public Trigger elevatorTopLimitReached = new Trigger(() -> getEncoderValue() > 5000);
  public Trigger elevatorNearBottom = new Trigger(()-> getEncoderValue() < 500);
  public Trigger limitReached = new Trigger(limitSwitch::get).negate();

  public Trigger elevatorAtL4;
  public ElevatorSubsystem() {
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorAtL4 = new Trigger(()-> elevatorPosition == ElevatorPosition.L4 || getEncoderValue() > 9800);

    setDefaultCommand(run(() -> {
      if(elevatorNearBottom.getAsBoolean() || manualMode == true){
        rightElevatorMotor.set(0); 
        leftElevatorMotor.set(0);
      }
      else{
        rightElevatorMotor.set(getMotorOutput()); 
        leftElevatorMotor.set(-getMotorOutput()); 
      }
      //System.out.println("default Command, encoder: " + getEncoderValue() + " setpoint: " + elevatorPosition.aim);
      //System.out.println("Default Command Running");
    }));
  }
  
  public enum ElevatorPosition { // Enums for elevator positions
    L0(1), // Aka ground
    L1(4813),
    L2(6158),
    L3(9238),
    L4(10200), //10084
    algae(10909),
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
      output = -maxElevatorSpeed;}
      
    return output;
    
  }

  public Command elevatorCommand(ElevatorPosition position){
    return new RunCommand(()-> {
      manualMode = false;
      elevatorPosition = position;
      rightElevatorMotor.set(getMotorOutput()); 
      leftElevatorMotor.set(-getMotorOutput());
      System.out.println(position);
      System.out.println("Output: " + getMotorOutput() + " Encoder: " + getEncoderValue());
    })
      .until(elevatorPositionReached)
      .andThen(runOnce(() -> {
      System.out.println("targetReached");
      System.out.println("encoder " + getEncoderValue());
      rightElevatorMotor.set(0);
      leftElevatorMotor.set(0);}))
      .withName("Position Elevator");
  }

  public Command elevatorL0Command(){
    return new RunCommand(() ->{
      elevatorPosition = ElevatorPosition.L0;
      rightElevatorMotor.set(getMotorOutput()); 
      leftElevatorMotor.set(-getMotorOutput());
      //System.out.println("encoder " + getEncoderValue());
    }).until(elevatorNearBottom).andThen(new RunCommand(() -> {
        System.out.println("slowDown");
        rightElevatorMotor.set(-0.05);
        leftElevatorMotor.set(0.05);}).until(elevatorPositionReached)
        .andThen(runOnce(() -> {shaftEncoder.reset();
                              System.out.println("end slowdown");
                              rightElevatorMotor.set(0);
                              leftElevatorMotor.set(0);})))
    .andThen(runOnce(() ->{
      rightElevatorMotor.set(0);
      leftElevatorMotor.set(0);
    }));
  }

  public Command manualElevatorUpCommand(CommandXboxController control){
    return run(()-> {
      manualMode = true;
      rightElevatorMotor.set(maxElevatorSpeed * 0.4);
      leftElevatorMotor.set(-maxElevatorSpeed * 0.4);
      System.out.println(getEncoderValue());})
      .withName("Manually Position Elevator Up");
  }

  public Command manualElevatorDownCommand(CommandXboxController control){
    return run(()-> {
      manualMode = true;
      rightElevatorMotor.set(-maxElevatorSpeed * 0.4);
      leftElevatorMotor.set(maxElevatorSpeed * 0.4);
      System.out.println(getEncoderValue());})
      .withName("Manually Position Elevator Down");
  }

  public Command resetEncoder(){
    return runOnce(() -> {shaftEncoder.reset();});
  }
}
