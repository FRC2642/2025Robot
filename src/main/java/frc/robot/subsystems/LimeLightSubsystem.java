// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.LimelightHelper;

public class LimeLightSubsystem extends SubsystemBase {
  public String limelightName;
  public double[] measuments;
  public reefPipes selectedPole;
  public double xSetpoint;
  public double ySetpoint;
  public double rotSetpoint;
  public double maxSpeed = 1;



  public PIDController xPidController = new PIDController(3, 0, 0);
  public PIDController yPidController = new PIDController(4, 0, 0);
  public PIDController rotPidController = new PIDController(0.1, 0, 0);
  public Trigger isAligned = new Trigger(()-> Math.abs(getOutputX()) < 0.1 && Math.abs(getOutputY()) < 0.15 && Math.abs(getOutputRot()) < 0.1);
  
  public LimeLightSubsystem(String limelightName) {
    this.limelightName = "limelight-" + limelightName;
    xSetpoint = -0.88;
    rotSetpoint = 0;
    
    setDefaultCommand(run(()->{
      
      // System.out.println(limelightName+ ": " + selectedPole);
      // System.out.println("y: " + measuments[0]);
      // System.out.println("rot: "+ measuments[4]);
    }));
  }
  public enum reefPipes{
    left(-0.17), //1.25
    center(0),
    right(0.17);
      public final double horizontalOffset;
      reefPipes(double horizontalOffset){
        this.horizontalOffset = horizontalOffset;
      }
  }
  public Command selectPoleCommand(reefPipes pole){
    return runOnce(()->{
      selectedPole = pole;
      ySetpoint = selectedPole.horizontalOffset;
      System.out.println("pole: "+pole);
      System.err.println("setpoint: "+pole.horizontalOffset);
    });
  }
  public void updateMeasurments(){
    measuments = LimelightHelper.getBotPose_TargetSpace(limelightName);
    //[tx, ty, tz, pitch, yaw, roll]
  }
  public double getOutputX(){
    updateMeasurments();
    double output = xPidController.calculate(measuments[2], xSetpoint);
    if (output > maxSpeed){
      output = maxSpeed;
    }
    if (output < -maxSpeed){
      output = -maxSpeed;
    }
    System.out.println("x: " + output);
    return output;
  }
  public double getOutputY(){
    updateMeasurments();
    double output = -yPidController.calculate(measuments[0], ySetpoint);
    if (output > maxSpeed){
      output = maxSpeed;
    }
    if (output < -maxSpeed){
      output = -maxSpeed;
    }
    System.out.println("y: " + output);
    return output;
  }
  public double getOutputRot(){
    updateMeasurments();
    double output = rotPidController.calculate(measuments[4], rotSetpoint);
    System.out.println("rot: " + output);
    return output;
  }
  public Command prints(){
    return runOnce(()->{
      updateMeasurments();
      System.out.println(limelightName);
      System.out.println("X: " + measuments[2]);
      System.out.println("Y: " + measuments[0]);
      System.out.println("Rot: " + measuments[4]);
    });
  }

  @Override
  public void periodic() {
  }
}
