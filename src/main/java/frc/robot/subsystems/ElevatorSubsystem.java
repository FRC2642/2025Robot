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
  public DigitalInput topLimitSwitch = new DigitalInput(6);

  public PIDController elevatorPID = new PIDController(1.8, 0.01, 0);
  public double maxElevatorSpeed = 2;
  public static ElevatorPosition elevatorPosition = ElevatorPosition.L0;
  public double startPosition;

  public boolean manualMode = true;

  public Trigger elevatorPositionReached = new Trigger(()-> Math.abs(getEncoderValue() - elevatorPosition.aim) < 25);
  public Trigger elevatorTopLimitReached = new Trigger(() -> getEncoderValue() > 10175);
  public Trigger elevatorNearBottom = new Trigger(()-> getEncoderValue() < 500);
  public Trigger elevatorNearTrigger = new Trigger(()-> Math.abs(getEncoderValue() - elevatorPosition.aim) < 500);
  public Trigger limitReached = new Trigger(limitSwitch::get).negate();
  public Trigger startingTrigger = new Trigger(()-> Math.abs(getEncoderValue() - startPosition) > 500);

  //RYLAN CHANGED
  public Trigger hitTop = new Trigger(topLimitSwitch::get).negate();

  public Trigger elevatorAtL4;
  public ElevatorSubsystem() {
    //rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    //leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorAtL4 = new Trigger(()-> elevatorPosition == ElevatorPosition.L4 || getEncoderValue() > 9800);

    setDefaultCommand(run(() -> {
      if(manualMode == false && elevatorPosition != ElevatorPosition.L0){
        rightElevatorMotor.set(getMotorOutput()); 
        leftElevatorMotor.set(-getMotorOutput());
      }
      else{
        rightElevatorMotor.set(0);
        leftElevatorMotor.set(0);
      }
      //System.out.println("default elevator encoder: " + getEncoderValue());
    }));
  }
  
  public enum ElevatorPosition { // Enums for elevator positions
    L0(1), // Aka ground
    L1(4813),
    L2(5337),
    L3(8570),
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
    double output = elevatorPID.calculate(getEncoderValue()/10000, elevatorPosition.aim/10000);
    if (output > maxElevatorSpeed){
      output = maxElevatorSpeed;}
    else if (output < -maxElevatorSpeed){
      output = -maxElevatorSpeed;}
      
    return output;
  }
  public Command elevatorL4AutoCommand(){
    return new RunCommand(()->{
      if (!hitTop.getAsBoolean()){
        rightElevatorMotor.set(maxElevatorSpeed * 0.4);
        leftElevatorMotor.set(-maxElevatorSpeed * 0.4);
      }
    }).until(hitTop.or(elevatorTopLimitReached)).andThen(runOnce(()->{
        rightElevatorMotor.set(0);
        leftElevatorMotor.set(0);
    }));
  }
  public Command elevatorDownAutoCommand(){
    return new RunCommand(()->{
      if (!limitReached.getAsBoolean()){
        rightElevatorMotor.set(-maxElevatorSpeed * 0.4);
        leftElevatorMotor.set(maxElevatorSpeed * 0.4);
      }
    }).until(limitReached).andThen(runOnce(()-> {
        rightElevatorMotor.set(0);
        leftElevatorMotor.set(0);
    }));
  }

  public Command superFancyElevatorCommand(ElevatorPosition position){
    return runOnce(()->{
        elevatorPosition = position;
        manualMode = false;
        startPosition = getEncoderValue();

    }).andThen(new RunCommand(()->{
        rightElevatorMotor.set(getMotorOutput() * ((Math.abs(startPosition - getEncoderValue()) + 20) / 500));
        leftElevatorMotor.set(-getMotorOutput() * ((Math.abs(startPosition - getEncoderValue()) + 20) / 500));
        //System.out.println("ramp up power output: " + getMotorOutput() * ((Math.abs(startPosition - getEncoderValue()) + 20) / 500));

    }).until(startingTrigger)).andThen(new RunCommand(()->{
        rightElevatorMotor.set(getMotorOutput()); 
        leftElevatorMotor.set(-getMotorOutput());
        //System.out.println("elevator moving with power output: " + getMotorOutput());
        //System.out.println("currectly at: " + getEncoderValue());

    })).until(elevatorNearTrigger).andThen(new RunCommand(()->{
        if (getEncoderValue() < elevatorPosition.aim){
          rightElevatorMotor.set(0.1);
          leftElevatorMotor.set(-0.1);
        } else{
          if(getEncoderValue() > elevatorPosition.aim){
            rightElevatorMotor.set(-0.1);
            leftElevatorMotor.set(0.1);
          }
        }

    })).until(elevatorPositionReached).andThen(runOnce(()->{
        //System.out.println("end");
        rightElevatorMotor  .set(0);
        leftElevatorMotor.set(0);
        
    }));
  }

  public Command manualElevatorUpCommand(CommandXboxController controller){
    return run(()-> {
      manualMode = true;
      if (!hitTop.getAsBoolean()){
        rightElevatorMotor.set(0.25);
        leftElevatorMotor.set(-0.25);
        //System.out.println("elevator encoder"+ getEncoderValue());
      }
      
    })
      .withName("Manually Position Elevator Up");
  }

  public Command manualElevatorDownCommand(CommandXboxController controller){
    return run(()-> {
      manualMode = true;
      if (!limitReached.getAsBoolean()){
        rightElevatorMotor.set(-0.25);
        leftElevatorMotor.set(0.25);
      //System.out.println(getEncoderValue());
      }
      })
      .withName("Manually Position Elevator Down");
  }

  public Command resetEncoder(){
    return runOnce(() -> {shaftEncoder.reset();});
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
