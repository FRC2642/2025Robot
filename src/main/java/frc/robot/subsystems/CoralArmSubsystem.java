// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class CoralArmSubsystem extends SubsystemBase {
  public TalonFX shootMotor = new TalonFX(Constants.ElevatorArm.SHOOT_MOTOR);
  public TalonFX rotateMotor = new TalonFX(Constants.ElevatorArm.ROTATE_ARM_MOTOR);  
  public CANrange beamBreak = new CANrange(26);
  private DutyCycleEncoder shaftEncoder= new DutyCycleEncoder(Constants.ElevatorArm.SHAFT_ENCODER_CHANNEL_A);
  

  private PIDController rotatePID = new PIDController(0.4, 0.2, 0);

  public double maxShootOutput= 0.25;
  public double maxRotateSpeed = 1;
  public ArmRotation armRot = ArmRotation.Default;
  public ShootSpeed shootSpeed = ShootSpeed.stop;
  public boolean intakeToggle = false;

  public Trigger RotationStateReached = new Trigger(() -> Math.abs(getEncoderValue() - armRot.rot) < 0.01);
  public Trigger IsSafeFromElevator = new Trigger(() -> getEncoderValue() > 0.3);
  public Trigger hasCoral = new Trigger(() -> beamBreak.getDistance().getValueAsDouble() < 0.08);
  public Trigger holdingAlgae = new Trigger(()-> intakeToggle == true);
  
  public CoralArmSubsystem() {
    rotateMotor.setNeutralMode(NeutralModeValue.Brake); 
    shootMotor.setNeutralMode(NeutralModeValue.Brake);
    
    setDefaultCommand(run(() -> {
      rotateMotor.set(0);
      /*
      if(getEncoderValue() < 0.086){
        rotateMotor.set(0);
      }else{
        rotateMotor.set(getrotateOutput());
      }
        */
      if (intakeToggle == false){
        shootSpeed = ShootSpeed.stop;
        shootMotor.set(0);
      }
      //System.out.println("default arm" + getEncoderValue());
      //System.out.println(intakeToggle);
      //System.out.println(beamBreak.getDistance().getValueAsDouble());
      //System.out.println(IsSafeFromElevator.getAsBoolean());
    })
    .withName("Idle"));
  }

  public enum ArmRotation{
    in(1),
    out(-1),
    Score(0.25),
    Default(0.085),
    Bottom(0.55),
    Safe(0.37);
      public final double rot;
      ArmRotation(double rotation) {
        this.rot = rotation;}
  }
  public enum ShootSpeed{
    superSlow(-0.25),
    intake(1), //1.25
    shoot(-1), //use only this
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


  public Command manualRotateCommand(ArmRotation position){
    double output = position.rot / 5;
    return run(()->{rotateMotor.set(output);});

  }

  public Command shootCommand(ShootSpeed speed){
    return new RunCommand(()-> {
      System.out.println("shoot Running");
      //intakeToggle = false;
      if (ElevatorSubsystem.elevatorPosition == ElevatorPosition.L4 || ElevatorSubsystem.elevatorPosition == ElevatorPosition.algae){
        shootSpeed = ShootSpeed.intake;
        shootMotor.set(shootSpeed.speed * maxShootOutput * 3);
      }else{
      shootMotor.set(speed.speed * maxShootOutput);}})
    .withName("Shoot Coral").onlyWhile(holdingAlgae.negate());
  }
  public Command toggleAlgaeIntake(){
    return runOnce(()-> {
      intakeToggle = !intakeToggle;
      System.out.println(("toggle: " + intakeToggle));
      if (intakeToggle == true){
        System.out.println("should be moving");
        shootSpeed = ShootSpeed.shoot;
        shootMotor.set(shootSpeed.speed * maxShootOutput * 0.7);
      }
      else{
        shootSpeed = ShootSpeed.stop;
        shootMotor.set(shootSpeed.speed * maxShootOutput);
      }
    });
  }
  public Command rotateCommand(ArmRotation armRotMode){
    return new RunCommand(()-> {
      //System.out.println("Rotating: " + getEncoderValue()); 
      armRot = armRotMode; rotateMotor.set(getrotateOutput() * 4);
      //System.out.println(IsSafeFromElevator.getAsBoolean());
    }).until(RotationStateReached)
    .andThen(runOnce(() -> rotateMotor.set(0)))
    .withName("Rotate Coral Arm");
  }

  public Command quickGetTheArmToSafty(){
    return new RunCommand(()->{
      armRot = ArmRotation.Bottom;
      rotateMotor.set(getrotateOutput() * 4);
    }).until(IsSafeFromElevator);
  }


  @Override
  public void periodic() {
  }
}
