// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JojoArmSubsystem extends SubsystemBase {
  public TalonFX rotateJojoMotor = new TalonFX(21);
  public TalonFX intakeJojoMotor = new TalonFX(20);
  private DutyCycleEncoder shaftEncoder = new DutyCycleEncoder(2);

  private PIDController rotatePID = new PIDController(0.4, 0, 0);

  public double maxRotateSpeed = 0.9;
  public double maxintakeSpeed = 0.8;
  public JojoRotation jojoRotation = JojoRotation.Default;
  public JojoIntake intakeMode = JojoIntake.stop;

  public Trigger RotationStateReached = new Trigger(() -> Math.abs(getEncoderValue() - jojoRotation.rot) < 0.01);
  public Trigger RotationStateNear = new Trigger(() -> Math.abs(getEncoderValue() - jojoRotation.rot) < 0.1);
  public Trigger CurrentSpike = new Trigger(() -> Math.abs(rotateJojoMotor.getStatorCurrent().getValue().magnitude()) > 10);

  public JojoArmSubsystem() {
    rotateJojoMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeJojoMotor.setNeutralMode(NeutralModeValue.Brake);

    setDefaultCommand(runOnce(()-> {intakeMode = JojoIntake.stop; intakeJojoMotor.set(0); rotateJojoMotor.set(0);})
    .andThen(run(() -> {}))
    .withName("Idle"));
  }

  public enum JojoRotation{
    Default(.1),
    Intake(.59);
    
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

  public double getEncoderValue() {
    double encoderValue = shaftEncoder.get();
    return encoderValue;
  }

  public double getrotateOutput() {
    double toRotate = rotatePID.calculate(getEncoderValue(), jojoRotation.rot);
    if (toRotate > maxRotateSpeed){
      toRotate = maxRotateSpeed;
    }
    else if (toRotate < -maxRotateSpeed){
      toRotate = -maxRotateSpeed;
    }
    return toRotate;
  }

  public Command intakeCommand(){
    return run(()-> {intakeMode = JojoIntake.intake;
                    intakeJojoMotor.set(intakeMode.intakeSpeed * maxintakeSpeed * 0.5);});
  }
  public Command extendCommand(){
    return new RunCommand(()-> {jojoRotation = JojoRotation.Intake; intakeMode = JojoIntake.intake;
      rotateJojoMotor.set(getrotateOutput() * 10);
      intakeJojoMotor.set(intakeMode.intakeSpeed * maxintakeSpeed);
      //System.out.println(Math.abs(rotateJojoMotor.getStatorCurrent().getValue().magnitude()));
      //System.out.println(CurrentSpike.getAsBoolean());
    }).until(RotationStateReached)
    .andThen(run(() -> {rotateJojoMotor.set(0); 
      intakeJojoMotor.set(intakeMode.intakeSpeed * maxintakeSpeed);}))
    .withName("Intake Jojo Arm");
  }
  
  public Command retractCommand(){
    return new RunCommand(()-> {jojoRotation = JojoRotation.Default;
    intakeMode = JojoIntake.stop; intakeJojoMotor.set(0); rotateJojoMotor.set(getrotateOutput() * 5);}).until(RotationStateReached)
    .andThen(runOnce(() -> rotateJojoMotor.set(0)));
  }



  @Override
  public void periodic() {
  }
}
